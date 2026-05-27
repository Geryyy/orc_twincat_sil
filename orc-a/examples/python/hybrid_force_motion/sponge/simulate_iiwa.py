import json
import os.path
import socket
import time

import mujoco
import mujoco.minimize
import mujoco.viewer
import numpy as np
import orcpy.core as oc
from orcpy.robots import inverse_kinematics_residual

start_point = np.array([-0.51903, 0, 0.8])
q0 = np.zeros(7)


def build_sponge_model():
    from MujocoEnvCreator import CustomEnvironment

    x_regal = -0.7
    sponge_geom_prefix = "iiwa/sponge/sponge_collision"

    env = CustomEnvironment("Iiwa with sponge")

    # Add Iiwa
    xml_robot = "environment_parts/iiwa_arc/iiwa.xml"
    iiwa = env.get_model_from_xml(xml_robot)
    quat = np.zeros(4)
    mujoco.mju_euler2Quat(quat, np.array([0, np.pi, np.pi / 2]), "xyz")
    env.add_model_to_arena(iiwa, "iiwa", [0, 0, 1.7], quat)
    sponge_model = env.get_model_from_xml("environment_parts/sponge/sponge_sensor.xml")
    env.add_model_to_site(sponge_model, "iiwa/gripper_attachment")
    sponge_collision = env.get_model_from_xml("environment_parts/sponge/sponge_collision.xml")
    env.add_model_to_site(sponge_collision, "iiwa/sponge/sponge_plate")

    # Add regal for drawing
    box_model = env.get_model_from_xml("environment_parts/Regal_braun/regal_braun.xml")
    quat = np.zeros(4)
    mujoco.mju_euler2Quat(quat, np.array([0, 0, np.pi / 2]), "xyz")
    env.add_model_to_arena(box_model, "Regal_braun", [x_regal, 0, 0], quat)

    xml_point = "environment_parts/handover_point/handover_point.xml"
    point_model = env.get_model_from_xml(xml_point)
    quat = np.zeros(4)
    mujoco.mju_euler2Quat(quat, np.array([0, -np.pi / 2, 0]), "xyz")
    # env.add_model_to_arena(point_model, "handover_point", start_point, quat, is_mocap=True)
    env.arena.worldbody.add(
        "site",
        name="handover_point",
        pos=start_point,
        size=[0.03],
        type="sphere",
        quat=quat,
        rgba=[1, 0, 0, 1],
    )

    model, data = env.compile_model()

    # Soften contact, needed as otherwise controller stability is hard to achieve
    geom_names = [
        model.geom(i).name
        for i in range(model.ngeom)
        if model.geom(i).name.startswith(sponge_geom_prefix)
    ]
    for geom_name in geom_names:
        model.geom(geom_name).priority = 1
        model.geom(geom_name).friction = np.zeros(3)
        model.geom(geom_name).solimp[0] = 0.0

    return model, data


def add_hybrid_force_controller(iiwa):
    param = oc.robots.robot7.HybridForceMotionParameter(True)
    iiwa.register_HybridForceMotionController(param)


if __name__ == "__main__":
    # Configuration
    sim_flag = True
    T_traj = 2
    UDP_IP = "127.0.0.1"  # PlotJuggler typically listens on localhost
    UDP_PORT = 9870  # Default port for PlotJuggler UDP plugin
    mjb_path = "models/presets/iiwa_hanging_with_sponge.mjb"

    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    oc.log.start_logging(oc.log.LogLevel.Debug)

    # Load Mujoco model
    mj_model, mj_data = build_sponge_model()
    Ts = mj_model.opt.timestep

    if os.path.exists(mjb_path) == False:
        oc.log.write_info(f"Saving model to {mjb_path}")
        mujoco.mj_saveModel(mj_model, mjb_path)

    # Controller parameters
    ctr_param = oc.robots.iiwa.IiwaContrParam(sim_flag)
    # Slow and fast normalization factors
    f_c_slow_norm = ctr_param.f_c_slow * (2 * Ts)
    # Joint, cartesian, and gravity compenation parameters
    js_param = oc.robots.robot7.JointCTParameter(ctr_param)
    ts_param = oc.robots.robot7.CartesianCTParameter(ctr_param)
    gc_param = oc.robots.robot7.GravityCompParameter(ctr_param.D_gc)

    iiwa = oc.robots.Iiwa(
        mjb_path,
        js_param,
        ts_param,
        gc_param,
        oc.Time(Ts),
        f_c_slow_norm,
        "iiwa/sponge/sponge_plate",
    )
    add_hybrid_force_controller(iiwa)

    # Trajectory servers
    com_server_robot = oc.robots.iiwa.TrajectoryServer(iiwa)

    # Calculate inverse dynamics
    def ik_fun(x):
        return inverse_kinematics_residual(
            x,
            mj_model,
            mj_data,
            mj_data.site("iiwa/sponge/sponge_plate"),
            mj_data.site("handover_point"),
            q0,
        )

    bounds = [mj_model.jnt_range[:, 0], mj_model.jnt_range[:, 1]]
    q_starting, _ = mujoco.minimize.least_squares(
        mj_data.qpos, ik_fun, bounds, verbose=1, gtol=1e-6
    )
    mj_data.qpos = q_starting
    mujoco.mj_forward(mj_model, mj_data)
    x_starting = np.zeros(7)
    x_starting[0:3] = mj_data.site("iiwa/sponge/sponge_plate").xpos
    mujoco.mju_mat2Quat(x_starting[3:7], mj_data.site("iiwa/sponge/sponge_plate").xmat)

    # Preare ready position
    iiwa.start(oc.Time(mj_data.time), q_starting, q_starting, oc.Time(T_traj))
    iiwa.set_q_d_nullspace(q_starting)

    framerate = 200
    frame_time = 1.0 / framerate
    print("frame_time: ", frame_time)
    mj_data.qpos = q_starting

    n_window = 10
    forces = np.zeros((n_window, 3))

    with mujoco.viewer.launch_passive(mj_model, mj_data) as viewer:
        # Close the viewer automatically after 30 wall-seconds.
        start = time.time()
        last_frame_time = time.time()

        # # Change the camera angle
        viewer.cam.lookat[2] = 0.8
        viewer.cam.distance = 3
        viewer.cam.elevation = -15
        viewer.cam.azimuth = 230

        while viewer.is_running():
            # Record the start time of each iteration for timekeeping
            step_start = time.time()

            # Poll trajectory servers for incoming data
            com_server_robot.poll()

            # Update control signals for Robot 0
            iiwa.set_q_act_filtered_derivatives(mj_data.qpos)
            forces = np.roll(forces, 1, axis=0)
            forces[0, :] = mj_data.sensordata[0:3]
            iiwa.robot_data.force_measurement = forces.mean(axis=0)
            ret = iiwa.update(oc.Time(mj_data.time))
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

            data_dict = {
                "tau_set": tau_set_robot.tolist(),
                "pose_act": iiwa.get_pose_act().tolist(),
                "pose_set": iiwa.get_pose_set().tolist(),
                "pose_err": (iiwa.get_pose_act() - iiwa.get_pose_set()).tolist(),
                "x_dot_act": iiwa.get_x_dot_act().tolist(),
                "x_dot_set": iiwa.get_x_dot_set().tolist(),
                "q_act": iiwa.get_q_act().tolist(),
                "force_d": iiwa.robot_data.force_d,
                "force_measurement": iiwa.robot_data.force_measurement[2],
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

            # Sleep to maintain the desired timestep
            time_until_next_step = mj_model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
