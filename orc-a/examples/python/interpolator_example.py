import matplotlib.pyplot as plt
import numpy as np
import orcpy.core as oc
import orcpy.robots as orco

# Show Cartesian pose interpolator if False
show_joint_spline_interpolator = True

T_traj = oc.Time(3.0)
mjb_path = orco.util_functions.default_model_path("iiwa_hanging.mjb")
iiwa = orco.Iiwa(mjb_path)
oc.log.start_logging()

##### Jointspace spline interpolator #####
if show_joint_spline_interpolator:
    q0 = np.zeros(7)
    q1 = np.ones(7)
    t0 = oc.Time(0.0)
    t1 = oc.Time(3.0)

    t_vec = oc.Time.convert_double_to_time_vector(np.linspace(0.0, 3.0, 30))
    interp = oc.robots.robot7.SplineJointInterpolator(q0, q1, t0, t1)
    interp.init(q0, q0, q0)

    points = np.zeros((7, len(t_vec)))
    for i, t in enumerate(t_vec):
        interp.update(t)
        points[:, i] = interp.get_point()

    plt.plot(np.linspace(0.0, 3.0, 30), points.T)
    plt.title("Jointspace spline interpolator")
    plt.xlabel("Time in s")
    plt.ylabel("Joint configuration")
    plt.legend()
    plt.show()

##### Cartesian pose interpolator #####
else:
    pose0 = np.zeros(7)
    pose1 = np.ones(7)
    t0 = oc.Time(0.0)
    t1 = oc.Time(3.0)

    t_vec = oc.Time.convert_double_to_time_vector(np.linspace(0.0, 3.0, 100))

    interp = oc.interpolator.CartesianPoseInterpolator(pose0, pose1, t0, t1)
    interp.init(pose0, np.zeros(6), np.ones(6))

    points = np.zeros((7, len(t_vec)))
    for i, t in enumerate(t_vec):
        interp.update(t)
        points[:, i] = interp.get_point()

    plt.plot(np.linspace(0.0, 3.0, 100), points.T)
    plt.title("Cartesian pose interpolator")
    plt.xlabel("Time in s")
    plt.ylabel("Pose")
    plt.legend()
    plt.show()
