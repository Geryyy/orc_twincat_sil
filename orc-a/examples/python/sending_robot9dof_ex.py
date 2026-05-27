import numpy as np
import orcpy.core as oc
import orcpy.robots as orco

simulation = True
oc.log.start_logging()
T_traj = oc.Time(8.0)

robot = orco.Robot9DOF.from_default_config(simulation)

q0 = np.zeros(7)
q1 = np.ones(7)

# Move iiwa to ones position and and back into candle position
robot.iiwa.move_jointspace(q1, robot.iiwa.time, T_traj, blocking_call=True)
robot.iiwa.move_jointspace(q0, robot.iiwa.time, T_traj, blocking_call=True)

# Move linear axis for 0.2m in both axis and back to original position
q_curr = robot.laxis.model.get_q_act()
q_set = q_curr + [0.15, 0.15]

robot.laxis.move_jointspace(q_set, robot.laxis.time, T_traj, N_pts=10, blocking_call=True)
robot.laxis.move_jointspace(q_curr, robot.laxis.time, T_traj, N_pts=10, blocking_call=True)
