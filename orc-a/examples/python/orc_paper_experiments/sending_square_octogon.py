import os
import time

import numpy as np
import orcpy.core as oc
import orcpy.robots as orco

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))


def get_circle_points(center, starting_point, N) -> np.ndarray:
    """Returns N points in a 2D plane forming a circle.

    Args:
        center (2D vector): center point of circle
        starting_point (2D vector): starting point on circle
        N (int): number of points to generate

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


def get_line_trajectory(start_pose, end_pose, force_traj, start_time, T_traj) -> np.ndarray:
    """Returns N points forming a line between start and end.

    Args:
        start_pose (xD vector): Starting point
        end_pose (xD vector): Ending point
        force_traj (N vector): Force trajectory
        start_time (float): Starting time of trajectory
        T_traj (float): Duration of trajectory

    Returns:
        np.ndarray: x*N array of points, where N is the length of forceF_traj and x the dimensionality of starting and end point
    """
    assert start_pose.shape == end_pose.shape, "Start and end point must have same dimensionality"
    N = 3
    points = np.zeros((N, start_pose.shape[0]))
    for i in range(N):
        alpha = i / (N - 1)
        points[i, :] = (1 - alpha) * start_pose + alpha * end_pose

    return points, oc.Time.convert_double_to_time_vector(
        np.linspace(start_time, start_time + T_traj, N)
    )


if __name__ == "__main__":
    simulation = True
    oc.log.start_logging()

    # Generate Iiwa instance
    mjb_path = os.path.join(REPO_ROOT, "models", "iiwa_hanging.mjb")
    ee_site_name = "iiwa_link_e"
    if simulation:
        T_traj = 10
        force_max = 2.0
        iiwa = orco.Iiwa(mjb_path, endeffector_site_name=ee_site_name)
    else:
        T_traj = 4
        force_max = 2.0
        local_ip_addr = "192.168.2.3"
        robot_ip_addr = "192.168.2.10"
        iiwa = orco.Iiwa(mjb_path, local_ip_addr, robot_ip_addr, endeffector_site_name=ee_site_name)
    time.sleep(0.5)
    T_traj_approach = 10

    # Move to starting pos
    q_starting = np.copy(iiwa.model.get_q_act())
    iiwa.set_nullspace(iiwa.time, iiwa.model.get_q_act())
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

    # Draw diamond shape
    diamond_side_length = 0.14
    diamond_half_diagonal = diamond_side_length / np.sqrt(2)
    force_vec = force_max * np.ones(3)
    x_center = np.copy(x_starting)
    x_center[2] += diamond_half_diagonal
    pose_starting = np.copy(x_starting)
    N_sides_arr = [4, 8]
    for N_sides in N_sides_arr:
        x_poses = np.zeros((N_sides, 3))
        for i in range(N_sides):
            x_ending = np.copy(x_starting)
            x_ending[0] = x_center[0] + diamond_half_diagonal * np.sin(
                i * np.pi * 2 / N_sides + 2 * np.pi / N_sides
            )
            x_ending[2] = x_center[2] - diamond_half_diagonal * np.cos(
                i * np.pi * 2 / N_sides + 2 * np.pi / N_sides
            )
            t_start = iiwa.time.to_sec()
            x_poses[i] = x_ending[0:3]
            points, time_vec = get_line_trajectory(
                pose_starting, x_ending, force_vec, t_start, T_traj
            )
            iiwa.send_hybrid_force_motion_trajectory(points, force_vec, time_vec)
            pose_starting = np.copy(x_ending)
            while iiwa.time.to_sec() < t_start + T_traj:
                time.sleep(0.1)
        # Shorter trajectory duration for octogon run
        T_traj = 3
