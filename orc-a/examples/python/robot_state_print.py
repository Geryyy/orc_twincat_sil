"""Shared helper for example scripts: pretty-print the current RobotState."""

import numpy as np

np.set_printoptions(precision=4, suppress=True, linewidth=120)


def print_robot_state(iiwa, target_label: str = "") -> None:
    """Pretty-print the current RobotState (actuals, setpoints, task-space)."""
    s = iiwa.state
    sep = "-" * 78
    header = f"Robot state  t={iiwa.time.to_sec():8.3f} s   status={s.status}"
    if target_label:
        header += f"   -> next target: {target_label}"
    print(sep)
    print(header)
    print(sep)
    print("  Joint space — actual:")
    print(f"    q_act        = {np.asarray(s.q_act)}")
    print(f"    q_dot_act    = {np.asarray(s.q_dot_act)}")
    print(f"    q_dotdot_act = {np.asarray(s.q_dotdot_act)}")
    print(f"    tau          = {np.asarray(s.tau)}")
    print("  Joint space — setpoints:")
    print(f"    q_set        = {np.asarray(s.q_set)}")
    print(f"    q_dot_set    = {np.asarray(s.q_dot_set)}")
    print(f"    q_dotdot_set = {np.asarray(s.q_dotdot_set)}")
    print(f"    q_d_NS       = {np.asarray(s.q_d_NS)}")
    print("  Task space — setpoints:")
    print(f"    x_set        = {np.asarray(s.x_set)}    (x,y,z, qw,qx,qy,qz)")
    print(f"    x_dot_set    = {np.asarray(s.x_dot_set)}")
    print(f"    x_dotdot_set = {np.asarray(s.x_dotdot_set)}")
    print(sep)
