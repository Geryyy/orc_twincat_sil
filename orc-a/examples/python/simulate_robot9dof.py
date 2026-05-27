import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
import orcpy.core as oc
import orcpy.robots as orco


def get_joint_ids(list_joint_names, mj_model):
    list_ids = []
    for list_name in list_joint_names:
        list_ids.append(mj_model.joint(list_name).id)
    return list_ids


def get_actuators_ids(list_actuator_names, mj_model):
    list_ids = []
    for list_name in list_actuator_names:
        list_ids.append(mj_model.actuator(list_name).id)
    return list_ids


def get_joint_and_actuator_ids(prefix, joint_names, actuator_names, mj_model):
    joint_ids = get_joint_ids([f"{prefix}/{joint}" for joint in joint_names], mj_model)
    actuator_ids = get_actuators_ids(
        [f"{prefix}/{actuator}" for actuator in actuator_names], mj_model
    )
    return joint_ids, actuator_ids


oc.log.start_logging()

# Compile model from MJB
mjb_path = orco.util_functions.default_model_path("models/presets/pascal_bernoulli.mjb")
generation_script_path = orco.util_functions.default_model_path(
    "generation_scripts/generate_pascal_bernoulli.py"
)

if not Path(mjb_path).exists():
    # Generate MJB from script
    import subprocess

    oc.log.write_info(f"MJB binary file not found. Running {generation_script_path} first!")
    subprocess.run(["python", generation_script_path])

mj_model = mujoco.MjModel.from_binary_path(mjb_path)
mj_data = mujoco.MjData(mj_model)

# Constants
Ts = mj_model.opt.timestep
sim_flag = True
T_traj = 2

# Define joint and actuator names for robot 0
robot0_joint_names = [f"iiwa/joint_{i}" for i in range(1, 8)]
robot0_actuator_names = [f"iiwa/torq_j{i}" for i in range(1, 8)]
joint_ids_robot0, actuator_ids_robot0 = get_joint_and_actuator_ids(
    "cage", robot0_joint_names, robot0_actuator_names, mj_model
)
# Define joint and actuator names for robot 1
robot1_joint_names = [f"iiwa_1/joint_{i}" for i in range(1, 8)]
robot1_actuator_names = [f"iiwa_1/torq_j{i}" for i in range(1, 8)]
joint_ids_robot1, actuator_ids_robot1 = get_joint_and_actuator_ids(
    "cage", robot1_joint_names, robot1_actuator_names, mj_model
)
# Define joint and actuator names for left L-axis
left_laxis_joint_names = ["x_left", "y_left"]
left_laxis_actuator_names = ["actuatorX_left", "actuatorY_left"]
joint_ids_laxis_left, actuator_ids_laxis_left = get_joint_and_actuator_ids(
    "cage", left_laxis_joint_names, left_laxis_actuator_names, mj_model
)
# Define joint and actuator names for right L-axis
right_laxis_joint_names = ["x_right", "y_right"]
right_laxis_actuator_names = ["actuatorX_right", "actuatorY_right"]
joint_ids_laxis_right, actuator_ids_laxis_right = get_joint_and_actuator_ids(
    "cage", right_laxis_joint_names, right_laxis_actuator_names, mj_model
)


# Robot classes
# Controller parameters
ctr_param = oc.robots.iiwa.IiwaContrParam(True)
# Slow and fast normalization factors
f_c_slow_norm = ctr_param.f_c_slow * (2 * Ts)

# Joint, cartesian, and gravity compenation parameters
js_param = oc.robots.robot7.JointCTParameter(ctr_param)
ts_param = oc.robots.robot7.CartesianCTParameter(ctr_param)
gc_param = oc.robots.robot7.GravityCompParameter(ctr_param.D_gc)

iiwa_mjb_path = orco.util_functions.default_model_path("iiwa_hanging.mjb")
iiwa1 = oc.robots.Iiwa(iiwa_mjb_path, js_param, ts_param, gc_param, oc.Time(Ts), f_c_slow_norm)
iiwa2 = oc.robots.Iiwa(iiwa_mjb_path, js_param, ts_param, gc_param, oc.Time(Ts), f_c_slow_norm)

K0 = 20 * np.identity(2)
laxis_mjb_path = orco.util_functions.default_model_path("linear_axis.mjb")
laxis1 = oc.robots.LinearAxis(laxis_mjb_path, oc.Time(Ts), K0)
laxis2 = oc.robots.LinearAxis(laxis_mjb_path, oc.Time(Ts), K0)

# Trajectory servers
com_server_robot0 = oc.robots.iiwa.TrajectoryServer(iiwa1)
com_server_robot1 = oc.robots.iiwa.TrajectoryServer(iiwa2, server_port=20001, client_port=21001)
com_server_l_axis_left = oc.robots.linear_axis.TrajectoryServer(
    laxis1, server_port=30002, client_port=31002
)
com_server_l_axis_right = oc.robots.linear_axis.TrajectoryServer(
    laxis2, server_port=40002, client_port=41002
)

# Start all robots
q0 = np.zeros(7)
t0 = oc.Time(mj_data.time)
iiwa1.start(t0, q0, q0, oc.Time(T_traj))
iiwa2.start(t0, q0, q0, oc.Time(T_traj))
q0_left = np.array([2.5, 0.5])
q0_right = np.array([0.5, 0.5])
laxis1.start(t0, q0_left, q0_left, oc.Time(T_traj))
laxis2.start(t0, q0_right, q0_right, oc.Time(T_traj))

framerate = 100
frame_time = 1.0 / framerate
print("frame_time: ", frame_time)
with mujoco.viewer.launch_passive(mj_model, mj_data) as viewer:
    mj_data.qpos[joint_ids_laxis_left] = q0_left
    mj_data.qpos[joint_ids_laxis_right] = q0_right

    # Close the viewer automatically after 30 wall-seconds.
    start = time.time()
    last_frame_time = time.time()
    # Change the camera angle
    viewer.cam.azimuth = -90
    viewer.cam.lookat = np.array([1.30851615, -0.24004286, 0.7547903])
    viewer.cam.distance = 8
    viewer.cam.elevation = -14

    while viewer.is_running():
        # Record the start time of each iteration for timekeeping
        step_start = time.time()

        # Poll trajectory servers for incoming data
        com_server_robot0.poll()
        com_server_robot1.poll()
        com_server_l_axis_left.poll()
        com_server_l_axis_right.poll()

        # Update control signals for Robot 0
        iiwa1.set_q_act_filtered_derivatives(mj_data.qpos[joint_ids_robot0])
        iiwa1.update(oc.Time(mj_data.time), False)
        mj_data.ctrl[actuator_ids_robot0] = iiwa1.get_tau_act()

        # Update control signals for Robot 1
        iiwa2.set_q_act_filtered_derivatives(mj_data.qpos[joint_ids_robot1])
        iiwa2.set_tau_sens(np.zeros(7))
        iiwa2.set_tau_motor(np.zeros(7))
        iiwa2.update(oc.Time(mj_data.time))
        mj_data.ctrl[actuator_ids_robot1] = iiwa2.get_tau_act()

        # Update control signals for left linear axis
        laxis1.set_q_act(mj_data.qpos[joint_ids_laxis_left])
        laxis1.update(oc.Time(mj_data.time))
        mj_data.ctrl[actuator_ids_laxis_left] = laxis1.get_tau_act()

        # Update control signals for right linear axis
        laxis2.set_q_act(mj_data.qpos[joint_ids_laxis_right])
        laxis2.update(oc.Time(mj_data.time))
        mj_data.ctrl[actuator_ids_laxis_right] = laxis2.get_tau_act()

        # Step the MuJoCo physics simulation
        mujoco.mj_step(mj_model, mj_data)

        # Send updated robot data to trajectory servers
        com_server_robot0.send_robot_data(oc.Time(mj_data.time))
        com_server_robot1.send_robot_data(oc.Time(mj_data.time))
        com_server_l_axis_left.send_robot_data(oc.Time(mj_data.time))
        com_server_l_axis_right.send_robot_data(oc.Time(mj_data.time))

        # Get robot states and errors for analysis
        iiwa_state_robot0 = oc.robots.robot7.RobotState(iiwa1, mj_data.time)
        iiwa_state_robot1 = oc.robots.robot7.RobotState(iiwa2, mj_data.time)

        # Calculate time remaining until next frame
        time_until_next_frame = frame_time - (time.time() - last_frame_time)
        if time_until_next_frame < 0:
            # If time until next frame is negative, reset last frame time and synchronize with viewer
            last_frame_time = time.time()
            viewer.sync()  # Pick up changes to the physics state, apply perturbations, update options from GUI

            # print("q_set (Robot 0):", arc_contr.get_q_set())
            # print("q_set (Robot 1):", arc_contr2.get_q_set())

        # Sleep to maintain the desired timestep
        time_until_next_step = mj_model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
