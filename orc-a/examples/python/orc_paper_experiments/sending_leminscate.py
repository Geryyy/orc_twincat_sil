"""
Send a lemniscate (figure-eight) Cartesian trajectory to either the Iiwa or
the Kinova Gen3 (selectable via ``--robot``). Requires a running simulation
or real robot server on the configured UDP endpoint.

While the trajectory executes, the RobotState stream received from the
simulator/robot is logged to a CSV in PlotJuggler format (matching the schema
of ``kinova_leminscate_plotjuggler.csv``).
"""

import argparse
import csv
import os
import threading
import time

import mujoco
import mujoco.minimize
import numpy as np
import orcpy.core as oc
import orcpy.robots as orco

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))


# Per-robot configuration -----------------------------------------------------

ROBOT_CONFIGS = {
    "iiwa": {
        "model_path": os.path.join(REPO_ROOT, "models", "iiwa_hanging.mjb"),
        "ee_site": "iiwa_link_e",
        "starting_configuration": np.array(
            [-0.01486022, 0.06610249, 0.02901589, -1.15850406, -0.08486216, 0.44967564, 0.14770002]
        ),
        "joint_limits_deg": (
            [-180, -128.0, -180, -147.8, -180, -120.3, -180],
            [180, 128.0, 180, 147.8, 180, 120.3, 180],
        ),
        "lemniscate_plane": "xz",  # x/z varies, y fixed (iiwa hanging)
    },
    "kinova": {
        "model_path": os.path.join(REPO_ROOT, "models", "kinova3.mjb"),
        "ee_site": "kinova3/gripping_point",
        # Match orc-kinova-gen3-kortex/orc_paper_experiments/sending_leminscate.py:
        # the on-robot example uses Kinova::q_home as the starting configuration
        # so the lemniscate is reproducible across sim and real hardware.
        "starting_configuration": np.array([0.0, 0.27, 3.14, -2.27, 0.0, 0.96, 1.57]),
        "joint_limits_deg": (
            [-180.0, -128.0, -180.0, -147.8, -180.0, -120.3, -180.0],
            [180.0, 128.0, 180.0, 147.8, 180.0, 120.3, 180.0],
        ),
        # Real experiment traces in the YZ-plane (x fixed at home).
        "lemniscate_plane": "yz",
        # Default lemniscate axes match the on-robot script (a=0.35, b=0.25).
        "lemniscate_a": 0.35,
        "lemniscate_b": 0.25,
    },
}


# Lemniscate geometry ---------------------------------------------------------


def get_lemniscate_points(n_points: int, a: float = 0.25, b: float = 0.15):
    """Generate N points on a lemniscate of Gerono (figure-eight)."""
    s = np.linspace(0.0, 1.0, n_points)
    t = 2.0 * np.pi * (3.0 * s**2 - 2.0 * s**3)
    u = a * np.sin(t)
    v = b * np.sin(t) * np.cos(t)
    return u, v


# Inverse kinematics ----------------------------------------------------------


def calculate_jointspace_trajectory(pose_traj, endeffector_rot, cfg, dof, starting_q):
    model = mujoco.MjModel.from_binary_path(cfg["model_path"])
    data = mujoco.MjData(model)
    site = data.site(cfg["ee_site"])

    def ik_fun(x, x_target, rot_target, reg_target):
        return orco.util_functions.inverse_kinematics_residual_posrot(
            x, model, data, site, x_target, rot_target, reg_target, reg=0.1
        )

    lo = np.deg2rad(np.array(cfg["joint_limits_deg"][0]))
    hi = np.deg2rad(np.array(cfg["joint_limits_deg"][1]))
    joint_limits = [lo, hi]

    n_points = pose_traj.shape[0]
    q_traj = np.zeros((n_points, dof))
    # Match orc-kinova-gen3-kortex sender: each IK call seeds from the
    # model's reference qpos and regularises toward starting_q (= q_home).
    # This ensures the joint trajectory matches the one executed on real
    # hardware exactly (as opposed to a warm-started variant which would
    # drift to a different IK branch).
    for i in range(n_points):
        ik_single_arg = lambda x, i=i: ik_fun(x, pose_traj[i], endeffector_rot, starting_q)
        q_traj[i], _ = mujoco.minimize.least_squares(
            data.qpos, ik_single_arg, joint_limits, verbose=0
        )

    return q_traj


# CSV logging -----------------------------------------------------------------


class StateLogger:
    """Polls robot.state and appends one row per sample in PlotJuggler format."""

    def __init__(self, robot, dof: int, path: str, period: float = 0.001):
        self._robot = robot
        self._dof = dof
        self._path = path
        self._period = period
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._n_rows = 0

    @staticmethod
    def _header(dof: int):
        cols = ["__time", "/dt"]
        for base in ("e_q", "q_act", "q_d", "q_dot_act", "q_dot_d", "q_dotdot_d", "q_raw", "tau"):
            cols += [f"/{base}[{i}]" for i in range(dof)]
        return cols

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join()

    def rows(self):
        return self._n_rows

    def _safe_arr(self, arr, n):
        if arr is None:
            return np.zeros(n)
        a = np.asarray(arr).ravel()
        if a.size < n:
            out = np.zeros(n)
            out[: a.size] = a
            return out
        return a[:n]

    def _run(self):
        dof = self._dof
        with open(self._path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(self._header(dof))

            last_wall = time.time()
            while not self._stop.is_set():
                st = self._robot.state
                t_wall = time.time()
                dt = t_wall - last_wall
                last_wall = t_wall

                q_act = self._safe_arr(getattr(st, "q_act", None), dof)
                q_d = self._safe_arr(getattr(st, "q_set", None), dof)
                q_dot_act = self._safe_arr(getattr(st, "q_dot_act", None), dof)
                q_dot_d = self._safe_arr(getattr(st, "q_dot_set", None), dof)
                q_dotdot_d = self._safe_arr(getattr(st, "q_dotdot_set", None), dof)
                # tau and q_raw are not exposed on the client-side RobotState;
                # emit zeros so the CSV schema still matches.
                tau = np.zeros(dof)
                q_raw = q_act.copy()
                e_q = q_d - q_act

                row = [f"{t_wall:.6f}", f"{dt:.9f}"]
                for arr in (e_q, q_act, q_d, q_dot_act, q_dot_d, q_dotdot_d, q_raw, tau):
                    row += [f"{v:.9f}" for v in arr]
                writer.writerow(row)
                self._n_rows += 1

                time.sleep(self._period)


# Main ------------------------------------------------------------------------


def build_robot(robot_name: str, model_path: str, simulation: bool):
    if robot_name == "iiwa":
        cls = orco.Iiwa
    elif robot_name == "kinova":
        cls = orco.Kinova
    else:
        raise ValueError(robot_name)

    if simulation:
        return cls(model_path)
    return cls(model_path, "192.168.2.3", "192.168.2.10")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", choices=["iiwa", "kinova"], default="iiwa")
    parser.add_argument(
        "--no-sim", action="store_true", help="Talk to real robot hardware instead of loopback sim."
    )
    parser.add_argument("--n-points", type=int, default=50, help="Number of lemniscate waypoints.")
    parser.add_argument(
        "--t-traj", type=float, default=9.0, help="Duration of the lemniscate execution [s]."
    )
    parser.add_argument(
        "--t-approach",
        type=float,
        default=5.0,
        help="Duration of the approach to the home configuration [s].",
    )
    parser.add_argument(
        "--a",
        type=float,
        default=None,
        help="Lemniscate first axis [m] (default: per-robot config).",
    )
    parser.add_argument(
        "--b",
        type=float,
        default=None,
        help="Lemniscate second axis [m] (default: per-robot config).",
    )
    parser.add_argument(
        "--csv",
        type=str,
        default=None,
        help="Output CSV path (default: <robot>_leminscate_plotjuggler.csv).",
    )
    parser.add_argument(
        "--log-period", type=float, default=0.001, help="Sampling period for the CSV logger [s]."
    )
    parser.add_argument(
        "--starting-q",
        type=str,
        default=None,
        help="Comma-separated starting joint configuration in radians, "
        "overriding the per-robot default.",
    )
    args = parser.parse_args()

    cfg = ROBOT_CONFIGS[args.robot]
    dof = 7
    n_points = args.n_points
    simulation = not args.no_sim

    np.set_printoptions(precision=3, suppress=True)
    oc.log.start_logging(oc.log.LogLevel.Info)

    robot = build_robot(args.robot, cfg["model_path"], simulation)

    # Give the receive thread a moment to ingest the first RobotState packets.
    time.sleep(0.5)

    if args.starting_q is not None:
        starting_q = np.fromstring(args.starting_q, sep=",")
        if starting_q.size != dof:
            raise ValueError(f"--starting-q must have {dof} entries, got {starting_q.size}")
    else:
        starting_q = cfg["starting_configuration"]
    if np.linalg.norm(robot.model.get_q_act() - starting_q) > 0.02:
        oc.log.write_info("Robot not at starting configuration — moving.")
        robot.move_jointspace(starting_q, robot.time, oc.Time(args.t_approach), blocking_call=True)

    # Cartesian trajectory relative to current EE pose.
    H0e = robot.model.get_current_H_0_e()
    ee_pos = H0e[:3, -1]
    ee_rot = H0e[:3, :3].flatten(order="C")

    a = args.a if args.a is not None else cfg.get("lemniscate_a", 0.25)
    b = args.b if args.b is not None else cfg.get("lemniscate_b", 0.15)
    u, v = get_lemniscate_points(n_points, a=a, b=b)
    pose_traj = np.zeros((n_points, 3))
    if cfg["lemniscate_plane"] == "xz":
        pose_traj[:, 0] = ee_pos[0] + u
        pose_traj[:, 1] = ee_pos[1]
        pose_traj[:, 2] = ee_pos[2] + v
    elif cfg["lemniscate_plane"] == "xy":
        pose_traj[:, 0] = ee_pos[0] + u
        pose_traj[:, 1] = ee_pos[1] + v
        pose_traj[:, 2] = ee_pos[2]
    elif cfg["lemniscate_plane"] == "yz":
        # Match orc-kinova-gen3-kortex sender: x fixed, y = u, z = v.
        pose_traj[:, 0] = ee_pos[0]
        pose_traj[:, 1] = ee_pos[1] + u
        pose_traj[:, 2] = ee_pos[2] + v
    else:
        raise ValueError(cfg["lemniscate_plane"])

    q_traj = calculate_jointspace_trajectory(pose_traj, ee_rot, cfg, dof, starting_q)

    csv_path = args.csv or os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        f"{args.robot}_leminscate_plotjuggler.csv",
    )
    logger = StateLogger(robot, dof, csv_path, period=args.log_period)
    logger.start()

    try:
        t_start = robot.time.to_sec() + 1.0
        t_arr = np.linspace(0.0, args.t_traj, n_points) + t_start
        robot.send_jointspace_trajectory(oc.Time.convert_double_to_time_vector(t_arr), q_traj)
        oc.log.write_info(f"Trajectory sent; logging to {csv_path}")

        # Wait for trajectory completion.
        deadline = t_start + args.t_traj + 0.5
        while robot.time.to_sec() < deadline:
            time.sleep(0.05)
    finally:
        logger.stop()
        oc.log.write_info(f"Wrote {logger.rows()} rows to {csv_path}")


if __name__ == "__main__":
    main()
