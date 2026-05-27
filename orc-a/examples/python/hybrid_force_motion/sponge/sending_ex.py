import time

import numpy as np
import orcpy.core as oc
import orcpy.robots as orco


def get_circle_points(center, starting_point, N) -> np.ndarray:
    """Returns N points in a 2D plane forming a circle.

    Args:
        center (2D vector): center point of circle
        starting_point (2D vector): starting point on circle
        N (ints): number of points to generate

    Returns:
        _type_: 2xN array of points
    """

    radius = np.linalg.norm(starting_point - center)
    starting_theta = np.arctan2(starting_point[1] - center[1], starting_point[0] - center[0])

    points = np.zeros((2, N))
    for i in range(N - 1):
        theta = starting_theta + 2 * np.pi * i / (N - 1)
        points[0, i] = center[0] + radius * np.cos(theta)
        points[1, i] = center[1] + radius * np.sin(theta)
    points[:, -1] = starting_point  # Close the circle
    return points


if __name__ == "__main__":
    simulation = True
    oc.log.start_logging()

    mjb_path = "models/presets/iiwa_hanging_with_sponge.mjb"
    ee_site_name = "iiwa/sponge/sponge_plate"
    if simulation:
        T_traj = oc.Time(3.0)
        iiwa = orco.Iiwa(mjb_path, endeffector_site_name=ee_site_name)
    else:
        T_traj = oc.Time(4)
        iiwa = orco.Iiwa(
            mjb_path, "192.168.2.3", "192.168.2.10", endeffector_site_name=ee_site_name
        )
    time.sleep(0.5)

    q_starting = iiwa.model.get_q_act()
    x_starting = iiwa.model.get_pose_act()

    # Prepare trajectory
    N = 4
    T_traj = 15
    force_max = 4.0

    center_point = np.copy(x_starting[1:3])
    center_point[1] -= 0.1

    plane_pts = get_circle_points(center_point, x_starting[1:3], N)
    pose_vec = x_starting * np.ones((N, 7))
    pose_vec[:, 1] = plane_pts[0, :]
    pose_vec[:, 2] = plane_pts[1, :]

    # import matplotlib.pyplot as plt
    # fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
    # ax.plot(pose_vec[:,0] * np.ones(N), pose_vec[:,1], pose_vec[:,2], 'o-')
    # plt.show()
    # exit()

    force_vec = force_max * np.ones(N)
    force_vec[0] = 0
    force_vec[1] = force_max / 2

    time_vec = np.linspace(iiwa.time.to_sec(), iiwa.time.to_sec() + T_traj, N)

    iiwa.send_hybrid_force_motion_trajectory(
        pose_vec, force_vec, oc.Time.convert_double_to_time_vector(time_vec)
    )
    # iiwa.move_jointspace(q_starting, iiwa.time + T_traj, oc.Time(3.0))
