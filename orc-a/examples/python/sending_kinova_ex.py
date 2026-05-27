## Example script to send jointspace commands to a Kinova robot in simulation
import time

import numpy as np
import orcpy.core as oc
import orcpy.robots as orco

oc.log.start_logging()

mjb_path = orco.util_functions.default_model_path("kinova3.mjb")
T_traj = oc.Time(8.0)
kinova = orco.Kinova(mjb_path)
time.sleep(0.5)

q_home = kinova.model.get_q_home()
q0 = kinova.model.get_q_act()
q1 = np.ones(7)

if np.linalg.norm(q_home - q0) < 0.1:
    kinova.move_jointspace(q1, kinova.time, T_traj)
else:
    kinova.move_jointspace(q_home, kinova.time, T_traj)
