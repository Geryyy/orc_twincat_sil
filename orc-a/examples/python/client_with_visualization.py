"""
Send an endless Lissajous Cartesian trajectory to the iiwa robot and visualize
the received RobotState as a digital twin in a passive MuJoCo viewer.

The script has two roles that run concurrently:
  1. Sender  — computes a Lissajous figure in Cartesian space, converts it to
               joint space via IK, and treats it as an endless function of
               absolute robot time.  Every second it resends a short,
               *overlapping* window of that loop, each starting slightly in the
               future.  Because a fresh window is always queued and takes over
               (with a C2-continuous hand-off) before the controller reaches
               the previous window's zero-velocity tail, the figure loops
               forever with no velocity jump / oscillation at the seam.
  2. Viewer  — continuously reads the RobotState streamed back by the robot /
               simulator and mirrors the actual joint positions into a passive
               MuJoCo window (no physics, kinematics only).

Usage — simulation (run simulate_iiwa.py first):
    python client_with_visualization.py

Usage — real robot:
    python client_with_visualization.py --robot-ip 192.168.2.10 --local-ip 192.168.2.3
"""

import argparse
import time

import mujoco
import mujoco.minimize
import mujoco.viewer
import numpy as np
import orcpy.core as oc
import orcpy.robots as orco

from robot_state_print import print_robot_state


# ---------------------------------------------------------------------------
# Lissajous geometry
# ---------------------------------------------------------------------------


def lissajous_points(
    n: int,
    a: float,
    b: float,
    freq_a: int,
    freq_b: int,
    delta: float = 0.0,
) -> tuple[np.ndarray, np.ndarray]:
    """Return (u, v) sample arrays for one period of a Lissajous figure.

    u(t) = a * sin(freq_a * t + delta)
    v(t) = b * sin(freq_b * t)

    delta=0 ensures the curve starts at (u, v) = (0, 0), i.e. the robot's
    current EE position, so there is no jump at trajectory start.
    """
    t = np.linspace(0.0, 2.0 * np.pi, n, endpoint=False)
    return a * np.sin(freq_a * t + delta), b * np.sin(freq_b * t)


# ---------------------------------------------------------------------------
# Inverse kinematics
# ---------------------------------------------------------------------------


def ik_jointspace(
    model_path: str,
    ee_site: str,
    pose_traj: np.ndarray,
    ee_rot: np.ndarray,
    starting_q: np.ndarray,
    joint_limits_deg: tuple,
) -> np.ndarray:
    """Solve IK for every Cartesian waypoint using MuJoCo's least-squares solver.

    Args:
        pose_traj: (N, 3) array of end-effector positions [m].
        ee_rot:    (9,) rotation matrix (row-major) kept constant across all waypoints.
        starting_q: regularisation target for IK (used as the rest posture).

    Returns:
        q_traj: (N, 7) joint configuration array.
    """
    model = mujoco.MjModel.from_binary_path(model_path)
    data = mujoco.MjData(model)
    site = data.site(ee_site)

    lo = np.deg2rad(np.array(joint_limits_deg[0]))
    hi = np.deg2rad(np.array(joint_limits_deg[1]))

    n = pose_traj.shape[0]
    q_traj = np.zeros((n, 7))

    for i in range(n):

        def residual(x, i=i):
            return orco.util_functions.inverse_kinematics_residual_posrot(
                x, model, data, site, pose_traj[i], ee_rot, starting_q, reg=0.1
            )

        q_traj[i], _ = mujoco.minimize.least_squares(
            data.qpos, residual, [lo, hi], verbose=0
        )

    return q_traj


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--robot-ip",
        default="127.0.0.1",
        help="iiwa IP address (default: 127.0.0.1 = simulation / loopback)",
    )
    parser.add_argument(
        "--local-ip",
        default="127.0.0.1",
        help="Local network interface IP (default: 127.0.0.1)",
    )
    parser.add_argument(
        "--n-points",
        type=int,
        default=60,
        help="Number of Lissajous waypoints (default: 60)",
    )
    parser.add_argument(
        "--t-traj",
        type=float,
        default=8.0,
        help="Lissajous trajectory duration [s] (default: 8.0)",
    )
    parser.add_argument(
        "--t-approach",
        type=float,
        default=5.0,
        help="Approach-move duration [s] (default: 5.0)",
    )
    parser.add_argument(
        "--a",
        type=float,
        default=0.12,
        help="Lissajous x-amplitude [m] (default: 0.12)",
    )
    parser.add_argument(
        "--b",
        type=float,
        default=0.08,
        help="Lissajous z-amplitude [m] (default: 0.08)",
    )
    parser.add_argument(
        "--freq-a",
        type=int,
        default=1,
        help="Lissajous x-frequency (default: 1)",
    )
    parser.add_argument(
        "--freq-b",
        type=int,
        default=2,
        help="Lissajous z-frequency (default: 2)",
    )
    args = parser.parse_args()

    # iiwa-specific constants (hanging configuration)
    # Intentionally using the meshless model for both IK and the viewer so the
    # digital-twin window is visually distinct from the simulate_iiwa.py window.
    model_path = orco.util_functions.default_model_path("iiwa_hanging.mjb")
    ee_site = "iiwa_link_e"
    starting_q = np.array(
        [-0.01486022, 0.06610249, 0.02901589, -1.15850406, -0.08486216, 0.44967564, 0.14770002]
    )
    joint_limits_deg = (
        [-180, -128.0, -180, -147.8, -180, -120.3, -180],
        [180, 128.0, 180, 147.8, 180, 120.3, 180],
    )

    oc.log.start_logging(oc.log.LogLevel.Info)

    # Connect to robot / simulator
    iiwa = orco.Iiwa(model_path, args.local_ip, args.robot_ip)
    time.sleep(0.5)  # let first UDP packets arrive so robot.time is valid

    # Move to starting configuration if needed
    if np.linalg.norm(iiwa.model.get_q_act() - starting_q) > 0.05:
        oc.log.write_info("Moving to starting configuration…")
        iiwa.move_jointspace(
            starting_q, iiwa.time, oc.Time(args.t_approach), blocking_call=True
        )

    # Build Lissajous waypoints in Cartesian space relative to current EE pose.
    # The figure is traced in the xz-plane (natural for the hanging iiwa).
    H0e = iiwa.model.get_current_H_0_e()
    ee_pos = H0e[:3, 3]
    ee_rot = H0e[:3, :3].flatten(order="C")

    u, v = lissajous_points(args.n_points, args.a, args.b, args.freq_a, args.freq_b)
    pose_traj = np.column_stack(
        [
            ee_pos[0] + u,                        # x varies
            np.full(args.n_points, ee_pos[1]),    # y fixed
            ee_pos[2] + v,                        # z varies
        ]
    )

    # Inverse kinematics
    oc.log.write_info(
        f"Computing IK for {args.n_points}-point Lissajous"
        f" (a={args.a} m, b={args.b} m, {args.freq_a}:{args.freq_b})…"
    )
    q_traj = ik_jointspace(
        model_path, ee_site, pose_traj, ee_rot, starting_q, joint_limits_deg
    )

    # ------------------------------------------------------------------
    # Overlapping-window streaming parameters.
    #
    # Each window we send spans WINDOW seconds but is replaced after only
    # SEND_INTERVAL seconds, so the controller only ever executes the first
    # ~SEND_INTERVAL of it before the next window hands over.  As long as
    # WINDOW is comfortably larger than SEND_INTERVAL the controller never
    # reaches the window's zero-velocity tail (which the interpolator appends
    # for safety) — that tail is what previously caused the velocity jump.
    # ------------------------------------------------------------------
    SEND_INTERVAL = 1.0  # resend a fresh window every second (robot time)
    WINDOW = 2.5         # seconds of trajectory covered by each window
    START_LEAD = 0.2     # start each window this far in the future (robot time)
    dt = args.t_traj / args.n_points  # waypoint spacing -> window sample density

    # The Lissajous figure is periodic with period t_traj.  Build a closed loop
    # (append q_traj[0] at phase = t_traj) so q_at(t) can map any absolute robot
    # time onto the trajectory by wrapping the phase and interpolating.
    phase_grid = np.append(
        np.linspace(0.0, args.t_traj, args.n_points, endpoint=False), args.t_traj
    )
    q_loop = np.vstack([q_traj, q_traj[0]])

    # Phase reference: absolute robot time at which the figure is at phase 0.
    t0 = iiwa.time.to_sec() + START_LEAD

    def q_at(t_abs: np.ndarray) -> np.ndarray:
        """Joint configuration(s) on the endless Lissajous at absolute time(s)."""
        phase = np.mod(np.asarray(t_abs, dtype=float) - t0, args.t_traj)
        return np.column_stack([np.interp(phase, phase_grid, q_loop[:, j]) for j in range(7)])

    n_win = max(2, round(WINDOW / dt) + 1)  # knots per window

    def send_window(now: float) -> None:
        """Send one overlapping window of the loop, beginning just after ``now``.

        The knots are snapped to the global ``t0 + k*dt`` grid, which keeps
        consecutive windows seam-free:
          * grid knots fall exactly on the original IK waypoints (no resampling
            error vs. the phase-arbitrary alternative), and
          * overlapping windows share identical knot times *and* values, so the
            controller's per-window spline fit — and hence the C2 hand-off
            between them — is consistent rather than wobbling each second.
        """
        k0 = int(np.ceil((now + START_LEAD - t0) / dt))
        t_arr = t0 + (k0 + np.arange(n_win)) * dt
        iiwa.send_jointspace_trajectory(
            oc.Time.convert_double_to_time_vector(t_arr), q_at(t_arr)
        )

    # Send the first window (non-blocking — controller executes autonomously)
    send_window(iiwa.time.to_sec())
    t_last_send = iiwa.time.to_sec()
    oc.log.write_info(
        f"Streaming endless Lissajous — {WINDOW:.1f}s windows resent every "
        f"{SEND_INTERVAL:.1f}s (period {args.t_traj:.1f}s), looping forever."
    )

    # ------------------------------------------------------------------
    # Digital-twin viewer
    # Mirror the actual joint positions received via RobotState into a
    # passive MuJoCo window.  No physics is stepped — only forward
    # kinematics is recomputed so the geometry tracks the real robot.
    # Between frames the viewer loop also checks whether the current
    # trajectory batch is about to expire and queues the next one.
    # ------------------------------------------------------------------
    mj_model = mujoco.MjModel.from_binary_path(model_path)
    mj_data = mujoco.MjData(mj_model)

    framerate = 60
    frame_dt = 1.0 / framerate

    with mujoco.viewer.launch_passive(mj_model, mj_data) as viewer:
        viewer.cam.lookat[2] = 0.75
        viewer.cam.distance = 3.0
        viewer.cam.elevation = -30

        oc.log.write_info("Digital-twin viewer running — close the window to exit.")

        t_last_print = iiwa.time.to_sec()
        while viewer.is_running():
            t_frame = time.time()

            # Resend an overlapping window every SEND_INTERVAL seconds, each
            # starting slightly in the future relative to the reported robot
            # time so it queues and hands over before the previous one ends.
            now = iiwa.time.to_sec()
            if now - t_last_send >= SEND_INTERVAL:
                send_window(now)
                t_last_send = now

            if now - t_last_print >= 5.0:
                print_robot_state(iiwa, "endless Lissajous")
                t_last_print = now

            q_act = iiwa.state.q_act
            if q_act is not None:
                mj_data.qpos[:7] = q_act
                # Forward kinematics only — no dynamics, no collision
                mujoco.mj_kinematics(mj_model, mj_data)
                viewer.sync()

            elapsed = time.time() - t_frame
            remaining = frame_dt - elapsed
            if remaining > 0:
                time.sleep(remaining)


if __name__ == "__main__":
    main()
