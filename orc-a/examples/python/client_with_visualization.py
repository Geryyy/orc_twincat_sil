"""
Send an endless Lissajous Cartesian trajectory to the iiwa robot and visualize
the received RobotState as a digital twin in a passive MuJoCo viewer.

The script has two roles that run concurrently:
  1. Sender  — computes a Lissajous figure in Cartesian space, converts it to
               joint space via IK, and continuously re-sends the trajectory so
               the figure loops forever without interruption.
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

    def send_batch(t_batch_start: float) -> float:
        """Send one period of the Lissajous trajectory starting at t_batch_start.

        Returns the absolute robot time at which this batch ends so the caller
        can schedule the next one.
        """
        print_robot_state(iiwa, f"Lissajous batch @ t={t_batch_start:.3f}s")
        t_arr = np.linspace(0.0, args.t_traj, args.n_points) + t_batch_start
        iiwa.send_jointspace_trajectory(
            oc.Time.convert_double_to_time_vector(t_arr), q_traj
        )
        return t_batch_start + args.t_traj

    # Send first batch (non-blocking — controller executes autonomously)
    t_batch_end = send_batch(iiwa.time.to_sec() + 1.0)
    oc.log.write_info(
        f"Trajectory sent — {args.n_points} pts over {args.t_traj:.1f} s, looping forever."
    )

    # How many seconds before the current batch ends to send the next one.
    # Keeps a trajectory queued on the controller at all times.
    SEND_AHEAD = 1.0

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

        while viewer.is_running():
            t_frame = time.time()

            # Queue next batch before the current one expires
            if iiwa.time.to_sec() >= t_batch_end - SEND_AHEAD:
                t_batch_end = send_batch(t_batch_end)

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
