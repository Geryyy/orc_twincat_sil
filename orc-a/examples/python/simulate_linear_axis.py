import sys
import time

import mujoco
import mujoco.viewer
import numpy as np
import orcpy.core as oc
import orcpy.robots as orco

if len(sys.argv) >= 2:
    mjb_path = sys.argv[1]
else:
    mjb_path = orco.util_functions.default_model_path("linear_axis.mjb")

oc.log.start_logging(oc.log.LogLevel.Debug)

# Load Mujoco model
mj_model = mujoco.MjModel.from_binary_path(mjb_path)
mj_data = mujoco.MjData(mj_model)
oc.log.write_info(f"Loaded model {mjb_path}")
Ts = mj_model.opt.timestep
sim_flag = True
T_traj = 2

# Controller parameters
K0 = 20 * np.identity(2)

# Start iiwa
laxis = oc.robots.LinearAxis(mjb_path, oc.Time(Ts), K0)
q0 = 0.5 * np.ones(2)
q1 = np.ones(2)
t0 = oc.Time(mj_data.time)
laxis.start(t0, q0, q0, oc.Time(1))

# Trajectory servers
com_server_robot = oc.robots.linear_axis.TrajectoryServer(laxis)

laxis.add_jointspace_trajectory(q0, q1, t0 + oc.Time(1.0), oc.Time(5.0))


framerate = 100
frame_time = 1.0 / framerate
print("frame_time: ", frame_time)
with mujoco.viewer.launch_passive(mj_model, mj_data) as viewer:
    # Close the viewer automatically after 30 wall-seconds.
    start = time.time()
    last_frame_time = time.time()
    # Change the camera angle
    viewer.cam.lookat[2] = 0.75
    viewer.cam.distance = 3
    viewer.cam.elevation = -30

    mj_data.qpos = q0

    while viewer.is_running():
        # Record the start time of each iteration for timekeeping
        step_start = time.time()

        # Poll trajectory servers for incoming data
        com_server_robot.poll()

        # Update control signals for Robot 0
        laxis.set_q_act(mj_data.qpos)
        laxis.set_q_dot_act(mj_data.qvel)
        ret = laxis.update(oc.Time(mj_data.time), False)

        if ret:
            vel_set_robot = laxis.get_tau_act()
            mj_data.qvel = vel_set_robot
        else:
            oc.log.write_error("Torque computation failed!")
            exit()

        # Step the MuJoCo physics simulation
        mujoco.mj_step(mj_model, mj_data)

        # Send updated robot data to trajectory servers
        com_server_robot.send_robot_data(oc.Time(mj_data.time))

        # Get robot states and errors for analysis
        laxis_state_robot = oc.robots.linear_axis.RobotState(laxis, mj_data.time)
        e_js_robot = laxis.get_joint_error()
        e_ts_robot = laxis.get_cartesian_error()

        # Calculate time remaining until next frame
        time_until_next_frame = frame_time - (time.time() - last_frame_time)
        if time_until_next_frame < 0:
            # If time until next frame is negative, reset last frame time and synchronize with viewer
            last_frame_time = time.time()
            viewer.sync()  # Pick up changes to the physics state, apply perturbations, update options from GUI

            # print("q_set (Robot 0):", iiwa.get_q_set())

        # Sleep to maintain the desired timestep
        time_until_next_step = mj_model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
