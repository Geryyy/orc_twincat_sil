import argparse
import json
import socket
import time

import mujoco
import mujoco.viewer
import numpy as np
import orcpy.core as oc
import orcpy.robots as orco
import orcpy.robots.util_functions as util_functions

# Configuration
UDP_IP = "127.0.0.1"  # PlotJuggler typically listens on localhost
UDP_PORT = 9870  # Default port for PlotJuggler UDP plugin

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

mjb_path = orco.util_functions.default_model_path("kinova3.mjb")


def main():
    # Arguments
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--no-sim", action="store_true", help="Talk to real robot hardware instead of loopback sim."
    )
    args = parser.parse_args()

    _host = "127.0.0.1"
    _port = 5000
    _sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    simulation = not args.no_sim
    mjb_path = orco.util_functions.default_model_path("kinova3.mjb")

    oc.log.start_logging(oc.log.LogLevel.Info)

    # Load Mujoco model
    mj_model = mujoco.MjModel.from_binary_path(mjb_path)
    mj_data = mujoco.MjData(mj_model)
    oc.log.write_info(f"Loaded model {mjb_path}")
    Ts = mj_model.opt.timestep

    # Controller parameters
    ctr_param = oc.robots.kinova.KinovaContrParam(simulation)
    js_param = oc.robots.robot7.JointPDPParameter(ctr_param.KP_PDP, ctr_param.KD_PDP)
    gc_param = oc.robots.robot7.GravityCompParameter(ctr_param.D_gc)
    kinova = oc.robots.Kinova(mjb_path, js_param, gc_param, oc.Time(Ts), "kinova3/gripping_point")

    if simulation:
        # Trajectory servers
        com_server_robot = oc.robots.kinova.TrajectoryServer(kinova)

        framerate = 100
        frame_time = 1.0 / framerate
        with mujoco.viewer.launch_passive(mj_model, mj_data) as viewer:
            # Close the viewer automatically after 30 wall-seconds.
            last_frame_time = time.time()
            # Change the camera angle
            viewer.cam.lookat[2] = 0.4
            viewer.cam.distance = 2
            viewer.cam.elevation = -30
            viewer.cam.azimuth = 145

            # Trajectory Setup
            q_home = kinova.get_q_home()
            mj_data.qpos = q_home
            T_traj = oc.Time(1)
            kinova.start(oc.Time(mj_data.time), q_home, q_home, T_traj)

            # qvel filter
            alpha = 0.1
            qvel_filtered = np.zeros(7)

            # Emergency mode for collisions
            emergency_mode = False

            while viewer.is_running():
                # Record the start time of each iteration for timekeeping
                step_start = time.time()

                kinova.set_q_act(mj_data.qpos)
                qvel_filtered = alpha * mj_data.qvel + (1 - alpha) * qvel_filtered
                kinova.set_q_dot_act(qvel_filtered)

                kinova.update(oc.Time(mj_data.time), False)
                tau_set_robot = kinova.get_tau_act()

                # Check for collisions
                if kinova.robot_data.collision_detected and not emergency_mode:
                    oc.log.write_warning(
                        "Emergency mode activated due to collision, trajectory server is not active!"
                    )
                    emergency_mode = True
                    kinova.add_jointspace_trajectory(
                        kinova.get_q_act(),
                        kinova.get_q_act(),
                        kinova.robot_data.t,
                        kinova.robot_data.t + 0.5,
                    )

                if not emergency_mode:
                    # Only poll in non-emergency mode
                    com_server_robot.poll()

                mj_data.ctrl = tau_set_robot

                com_server_robot.send_robot_data(oc.Time(mj_data.time))

                # Get robot states and errors for analysis
                data_dict = {
                    "q_act": kinova.get_q_act().tolist(),
                    "q_set": kinova.get_q_set().tolist(),
                    "tau_set": tau_set_robot.tolist(),
                    "e_js_robot": kinova.get_joint_error().tolist(),
                    "e_ts_robot": kinova.get_cartesian_error().tolist(),
                    "pose_act": kinova.get_pose_act().tolist(),
                    "pose_set": kinova.get_pose_set().tolist(),
                    "x_dot_act": kinova.get_x_dot_act().tolist(),
                    "x_dot_set": kinova.get_x_dot_set().tolist(),
                }
                message = json.dumps(data_dict)
                # Send via UDP
                sock.sendto(message.encode("utf-8"), (UDP_IP, UDP_PORT))

                mujoco.mj_step(mj_model, mj_data)

                time_until_next_frame = frame_time - (time.time() - last_frame_time)
                if time_until_next_frame < 0:
                    last_frame_time = time.time()
                    viewer.sync()

                time_until_next_step = mj_model.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)

    else:
        import orcpy.robots.com.KinovaConnection as KinovaConnection

        # On real robot
        np.set_printoptions(precision=2, suppress=True)
        T_traj = oc.Time(10)
        with KinovaConnection() as conn:
            conn.set_robot(kinova)

            print(kinova.get_q_act())
            # Check if in home pose
            q0 = util_functions.wrap_to_pi(
                np.deg2rad(np.array([act.position for act in conn.state.actuators]))
            )
            q_home = kinova.get_q_home()
            if np.linalg.norm(util_functions.wrap_to_pi(q0 - q_home)) > 0.02:
                print("ERROR: Robot is not in home pose!")
                exit()

            # Set torque mode only after check and start controller
            conn.set_torque_mode()
            kinova.start(conn.t, q_home, q_home, T_traj)
            while True:
                # Kortex API update
                conn.update()
                time.sleep(0.1)


if __name__ == "__main__":
    main()
