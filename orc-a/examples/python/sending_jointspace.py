import time

import numpy as np
import orcpy.core as oc
import orcpy.robots as orco

from robot_state_print import print_robot_state

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

while True:
    print_robot_state(iiwa, "q1 = ones(7)")
    iiwa.move_jointspace(q1, iiwa.time, T_traj, blocking_call=True)
    time.sleep(1.0)
    print_robot_state(iiwa, "q0 = zeros(7)")
    iiwa.move_jointspace(q0, iiwa.time, T_traj, blocking_call=True)
    time.sleep(1.0)
