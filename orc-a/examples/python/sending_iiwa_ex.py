import sys
import time

import numpy as np
import orcpy.core as oc
import orcpy.robots as orco

simulation = True
use_quaternion = False  # If set to True, move_taskspace will be called using quaternions. Either way the same trajectory is sent.

oc.log.start_logging()

mjb_path = orco.util_functions.default_model_path("iiwa_hanging.mjb")
if simulation:
    T_traj = oc.Time(3.0)
    iiwa = orco.Iiwa(mjb_path)
else:
    T_traj = oc.Time(4.0)
    iiwa = orco.Iiwa(mjb_path, "192.168.2.3", "192.168.2.10")
time.sleep(0.5)

q0 = np.zeros(7)
q1 = np.ones(7)

if len(sys.argv) == 2:
    if sys.argv[1] == "j":
        # Move JS
        q_act = iiwa.model.get_q_act()

        if abs(q_act[0]) < 0.1:
            t, q, _ = iiwa.move_jointspace(q1, iiwa.time, T_traj)
        else:
            t, q, _ = iiwa.move_jointspace(q0, iiwa.time, T_traj)
    elif sys.argv[1] == "t":
        # Move TS
        H_0_e = iiwa.model.get_current_H_0_e()
        d_pos = np.array([0, 0, +0.1])
        H_0_e[0:3, 3] = H_0_e[0:3, 3] + d_pos
        # iiwa.set_nullspace(iiwa.time, iiwa.model.get_q_act())
        iiwa.move_taskspace(H_0_e, iiwa.time, T_traj)
    elif sys.argv[1] == "n":
        iiwa.set_nullspace(iiwa.time, iiwa.model.get_q_act())

    exit()


# Do whole sequence
iiwa.move_jointspace(q1, iiwa.time, T_traj)
time.sleep(T_traj.to_sec() + 1)

d_pos_array = [np.array([0, 0, +0.15]), np.array([0, 0, -0.15])]
for d_pos in d_pos_array:
    iiwa.set_nullspace(iiwa.time, iiwa.model.get_q_act())
    H_0_e = iiwa.model.get_current_H_0_e()
    H_0_e[0:3, 3] = H_0_e[0:3, 3] + d_pos

    if use_quaternion:
        # Converts homogeneous transformation from the robot model to a quaternion. Both MuJoCo and ORC use a scalar first
        # convention, i.e., (w, x, y, z), for quaternions.
        import scipy.spatial.transform as sptrans

        rot = sptrans.Rotation.from_matrix(H_0_e[0:3, 0:3])
        quat = rot.as_quat(
            scalar_first=True
        )  # Scipy however defaults to scalar last, make sure the scalar_first flag is set to False.
        pose = np.hstack((H_0_e[0:3, 3], quat)).T
        iiwa.move_taskspace(pose, iiwa.time, T_traj)
    else:
        # Move using homogeneous transformation
        iiwa.move_taskspace(H_0_e, iiwa.time, T_traj)

    time.sleep(T_traj.to_sec() + 1)

iiwa.move_jointspace(q0, iiwa.time, T_traj)
