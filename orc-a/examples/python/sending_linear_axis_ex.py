import orcpy.core as oc
import orcpy.robots as orco

simulation = True

oc.log.start_logging()

laxis = orco.LinearAxis.from_default_config(simulation)

q_act = laxis.model.get_q_act()
q_set = q_act + [0.0, 0.1]
print(f"Moving from {q_act} to {q_set}")

t, joints, joints_d = laxis.move_jointspace(q_set, laxis.time, oc.Time(5.0), N_pts=10)
