import time

import numpy as np
import orcpy.core as oc
import orcpy.robots as orco

simulation = False

oc.log.start_logging()

mjb_path = orco.util_functions.default_model_path("iiwa_hanging.mjb")
T_traj = oc.Time(5.0)

if simulation:
    iiwa = orco.Iiwa(mjb_path)
else:
    iiwa = orco.Iiwa(mjb_path, "192.168.1.3", "192.168.1.10")
time.sleep(0.5)

q0 = np.zeros(7)
q1 = np.ones(7)

np.set_printoptions(precision=4, suppress=True, linewidth=120)


def print_robot_state(iiwa, target_label: str) -> None:
    """Pretty-print the current RobotState before sending the next trajectory."""
    s = iiwa.state
    sep = "-" * 78
    print(sep)
    print(f"Robot state  t={iiwa.time.to_sec():8.3f} s   status={s.status}   -> next target: {target_label}")
    print(sep)
    print("  Joint space — actual:")
    print(f"    q_act       = {np.asarray(s.q_act)}")
    print(f"    q_dot_act   = {np.asarray(s.q_dot_act)}")
    print(f"    q_dotdot_act= {np.asarray(s.q_dotdot_act)}")
    print(f"    tau         = {np.asarray(s.tau)}")
    print("  Joint space — setpoints:")
    print(f"    q_set       = {np.asarray(s.q_set)}")
    print(f"    q_dot_set   = {np.asarray(s.q_dot_set)}")
    print(f"    q_dotdot_set= {np.asarray(s.q_dotdot_set)}")
    print(f"    q_d_NS      = {np.asarray(s.q_d_NS)}")
    print("  Task space — setpoints:")
    print(f"    x_set       = {np.asarray(s.x_set)}    (x,y,z, qw,qx,qy,qz)")
    print(f"    x_dot_set   = {np.asarray(s.x_dot_set)}")
    print(f"    x_dotdot_set= {np.asarray(s.x_dotdot_set)}")
    print(sep)


while True:
    print_robot_state(iiwa, "q1 = ones(7)")
    iiwa.move_jointspace(q1, iiwa.time, T_traj, blocking_call=True)
    time.sleep(1.0)
    print_robot_state(iiwa, "q0 = zeros(7)")
    iiwa.move_jointspace(q0, iiwa.time, T_traj, blocking_call=True)
    time.sleep(1.0)
