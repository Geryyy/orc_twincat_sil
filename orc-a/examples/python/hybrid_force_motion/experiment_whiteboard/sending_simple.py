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

    mjb_path = "models/presets/iiwa_hanging_adapter_pen.mjb"
    ee_site_name = "iiwa/adapter_mini40/pen/pen_tip"
    if simulation:
        force_max = 2.0
        iiwa = orco.Iiwa(mjb_path, endeffector_site_name=ee_site_name)
    else:
        force_max = 2.0
        local_ip_addr = "192.168.2.3"
        robot_ip_addr = "192.168.2.10"
        iiwa = orco.Iiwa(mjb_path, local_ip_addr, robot_ip_addr, endeffector_site_name=ee_site_name)

    time.sleep(0.5)
    # Prepare trajectory
    N = 4
    T_traj = 15
    T_traj_approach = 10

    # # Move to starting pos
    iiwa.set_nullspace(iiwa.time, iiwa.model.get_q_act())
    print(iiwa.model.get_q_act())
    time.sleep(0.5)

    # Approach point
    t_start = iiwa.time.to_sec() + 1
    x_starting = np.copy(iiwa.model.get_pose_act())
    pose_vec = x_starting * np.ones((3, 7))
    force_vec = force_max * np.ones(3)
    force_vec[0] = 0
    force_vec[1] = force_max / 2
    time_vec = np.linspace(t_start, t_start + T_traj_approach, 3)
    iiwa.send_hybrid_force_motion_trajectory(
        pose_vec, force_vec, oc.Time.convert_double_to_time_vector(time_vec)
    )
    time.sleep(T_traj_approach)

    t_start = iiwa.time.to_sec() + 1
    N_traj = 4
    distance = -0.2
    # iiwa.set_nullspace(iiwa.time, iiwa.model.get_q_act())
    # time.sleep(0.5)
    # x_starting = np.copy(iiwa.model.get_pose_act())
    pose_vec = x_starting * np.ones((N_traj, 7))
    for i in range(N_traj):
        pose_vec[i, 2] += distance * i / (N_traj - 1)
    force_vec = force_max * np.ones(N_traj)
    time_vec = np.linspace(t_start, t_start + T_traj, N_traj)
    iiwa.send_hybrid_force_motion_trajectory(
        pose_vec, force_vec, oc.Time.convert_double_to_time_vector(time_vec)
    )

    if False:
        traj = oc.robots.robot7.HybridForceMotionTrajectory(
            pose_vec, force_vec, oc.Time.convert_double_to_time_vector(time_vec)
        )
        force_traj = []
        pose_traj = []
        x_dot = []
        traj.init()
        fine_times = np.linspace(t_start, t_start + T_traj, 30)
        for times in fine_times:
            traj.update(oc.Time(times))
            force_traj.append(traj.get_force())
            pose_traj.append(traj.get_pose())
            x_dot.append(traj.get_x_dot())

        import matplotlib.pyplot as plt

        plt.subplot(3, 1, 1)
        plt.plot(fine_times, pose_traj, ".")
        plt.subplot(3, 1, 2)
        plt.plot(fine_times, force_traj, ".")
        plt.subplot(3, 1, 3)
        plt.plot(fine_times, x_dot, ".")
        plt.show()
        exit()
