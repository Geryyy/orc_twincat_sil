import json
import socket
import sys
import time

import mujoco
import mujoco.viewer
import numpy as np
import orcpy.core as oc
import orcpy.robots as orco

# Configuration
UDP_IP = "127.0.0.1"  # PlotJuggler typically listens on localhost
UDP_PORT = 9870  # Default port for PlotJuggler UDP plugin

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


if len(sys.argv) >= 2:
    mjb_path = sys.argv[1]
else:
    mjb_path = orco.util_functions.default_model_path("iiwa_hanging_meshed.mjb")

oc.log.start_logging()

# Load Mujoco model
mj_model = mujoco.MjModel.from_binary_path(mjb_path)
mj_data = mujoco.MjData(mj_model)
oc.log.write_info(f"Loaded model {mjb_path}")
Ts = mj_model.opt.timestep
sim_flag = True
T_traj = 2

# Controller parameters
ctr_param = oc.robots.iiwa.IiwaContrParam(sim_flag)

# Slow and fast normalization factors
f_c_slow_norm = ctr_param.f_c_slow * (2 * Ts)

# Joint, cartesian, and gravity compenation parameters
js_param = oc.robots.robot7.JointCTParameter(ctr_param)
ts_param = oc.robots.robot7.CartesianCTParameter(ctr_param)
gc_param = oc.robots.robot7.GravityCompParameter(ctr_param.D_gc)

# Start iiwa
iiwa = oc.robots.Iiwa(
    mjb_path, js_param, ts_param, gc_param, oc.Time(Ts), f_c_slow_norm, "iiwa/iiwa_link_e"
)
q0 = np.zeros(7)
q1 = np.ones(7)
t0 = oc.Time(mj_data.time)
iiwa.start(t0, q0, q0, oc.Time(T_traj))

# Trajectory servers
com_server_robot = oc.robots.iiwa.TrajectoryServer(iiwa)


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

    while viewer.is_running():
        # Record the start time of each iteration for timekeeping
        step_start = time.time()

        # Poll trajectory servers for incoming data
        com_server_robot.poll()

        # Update control signals for Robot 0
        iiwa.set_q_act_filtered_derivatives(mj_data.qpos)
        ret = iiwa.update(oc.Time(mj_data.time), False)

        if ret:
            tau_set_robot = iiwa.get_tau_act()
            mj_data.ctrl = tau_set_robot
        else:
            oc.log.write_error("Torque computation failed!")
            exit()

        # Step the MuJoCo physics simulation
        mujoco.mj_step(mj_model, mj_data)

        # Send updated robot data to trajectory servers
        com_server_robot.send_robot_data(oc.Time(mj_data.time))

        # Get robot states and errors for analysis
        iiwa_state_robot = oc.robots.robot7.RobotState(iiwa, mj_data.time)
        e_js_robot = iiwa.get_joint_error()
        e_ts_robot = iiwa.get_cartesian_error()

        data_dict = {
            "q_act": iiwa.get_q_act().tolist(),
            "q_set": iiwa.get_q_set().tolist(),
            "tau_set": tau_set_robot.tolist(),
            "e_js_robot": e_js_robot.tolist(),
            "e_ts_robot": e_ts_robot.tolist(),
            "pose_act": iiwa.get_pose_act().tolist(),
            "pose_set": iiwa.get_pose_set().tolist(),
            "x_dot_act": iiwa.get_x_dot_act().tolist(),
            "x_dot_set": iiwa.get_x_dot_set().tolist(),
        }
        message = json.dumps(data_dict)
        # Send via UDP
        sock.sendto(message.encode("utf-8"), (UDP_IP, UDP_PORT))

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
