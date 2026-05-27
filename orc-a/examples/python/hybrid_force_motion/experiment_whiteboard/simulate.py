import json
import socket
import time

import mujoco
import mujoco.minimize
import mujoco.viewer
import numpy as np
import orcpy.core as oc
from orcpy.robots import inverse_kinematics_residual

whiteboard_pos = np.array([-0.4, -0.5, 1.1])
whiteboard_size = np.array([0.15, 0.01, 0.2])


def build_model_with_MMM(add_whiteboard=True, discard_visuals=False):
    """Builds MuJoCo model using MujocoMeshManager

    Args:
        discard_visuals (bool, optional): Sets corresponding flag in MuJoCo compiler. Defaults to False.

    Returns:
        MjModel, MjData: Compiled MuJoCo model and data
    """
    from MujocoEnvCreator import CustomEnvironment

    env = CustomEnvironment("Iiwa (Whiteboard experiment)")

    # Add Iiwa
    xml_robot = "environment_parts/iiwa_arc/iiwa.xml"
    iiwa = env.get_model_from_xml(xml_robot)
    quat = np.zeros(4)
    mujoco.mju_euler2Quat(quat, np.array([0, np.pi, np.pi / 2]), "xyz")
    env.add_model_to_arena(iiwa, "iiwa", [0, 0, 1.7], quat)

    # Add adapter and pen
    adapter_model = env.get_model_from_xml("environment_parts/adapter_mini40/adapter_mini40.xml")
    env.add_model_to_site(adapter_model, "iiwa/gripper_attachment")

    pen_model = env.get_model_from_xml("environment_parts/pen/pen_force_sensor.xml")
    env.add_model_to_site(pen_model, "iiwa/adapter_mini40/adapter_attachment")

    if False:
        # Add regal for drawing
        box_model = env.get_model_from_xml("environment_parts/Regal_braun/regal_braun.xml")
        quat = np.zeros(4)
        mujoco.mju_euler2Quat(quat, np.array([0, 0, np.pi / 2]), "xyz")
        env.add_model_to_arena(box_model, "Regal_braun", [x_regal, 0, 0], quat)

    if add_whiteboard:
        # Add whiteboard
        quat = np.zeros(4)
        mujoco.mju_euler2Quat(quat, np.array([0, 0, 0]), "xyz")
        env.arena.worldbody.add(
            "geom",
            name="whiteboard",
            pos=whiteboard_pos,
            size=whiteboard_size,
            type="box",
            quat=quat,
            rgba=[1, 1, 1, 1],
        )

        # Add starting point
        starting_point_pos = whiteboard_pos.copy()
        starting_point_pos[1] += whiteboard_size[1]
        starting_point_pos[2] -= whiteboard_size[2] / 2
        mujoco.mju_euler2Quat(quat, np.array([np.pi / 2, 0, np.pi]), "xyz")
        env.arena.worldbody.add(
            "site",
            name="starting_point",
            pos=starting_point_pos,
            size=[0.005],
            type="sphere",
            quat=quat,
            rgba=[1, 0, 0, 1],
        )

    model, data = env.compile_model(discard_visuals=discard_visuals)

    # Soften contact, needed as otherwise controller stability is hard to achieve
    geom_name = "iiwa/adapter_mini40/pen/pen_collision_tip"
    model.geom(geom_name).priority = 1
    model.geom(geom_name).friction = np.zeros(3)
    model.geom(geom_name).solimp = [0.0, 0.9, 0.02, 0.5, 2]
    model.geom(geom_name).solref = [0.05, 1.0]
    return model, data


def visualize_contact(mj_data, viewer, contact, trail_idx):
    trail_max_length = 10_000

    fromto = np.zeros(6, dtype=np.float64)
    regal_back_id = mj_data.geom("whiteboard").id
    pen_tip_id = mj_data.geom("iiwa/adapter_mini40/pen/pen_collision_tip").id
    distance = mujoco.mj_geomDistance(mj_model, mj_data, regal_back_id, pen_tip_id, 10, fromto)
    if distance <= 0.001:
        if len(contact) == 0:
            contact = np.array(fromto[3:])
        else:
            idx = trail_idx % trail_max_length
            geom = viewer.user_scn.geoms[idx]

            mujoco.mjv_initGeom(
                geom,
                type=mujoco.mjtGeom.mjGEOM_CAPSULE,
                size=[0.004, 0, 0],
                pos=[0, 0, 0],
                mat=np.eye(3).flatten(),
                rgba=np.array([0.8, 0, 0, 1]),
            )
            mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, 0.002, contact, fromto[3:])
            trail_idx += 1
            viewer.user_scn.ngeom = min(trail_idx, trail_max_length)
            contact = fromto[3:]
    else:
        contact = []

    return mj_data, viewer, contact, trail_idx


if __name__ == "__main__":
    # Configuration
    UDP_IP = "127.0.0.1"  # PlotJuggler typically listens on localhost
    UDP_PORT = 9870  # Default port for PlotJuggler UDP plugin
    mjb_path = "models/presets/iiwa_hanging_adapter_pen.mjb"
    endeffector_site_name = "iiwa/adapter_mini40/pen/pen_tip"

    # Build meshless MJB and save it for use in ORC.
    # The model doesn't necessarily have to be meshless. It just takes up less memory.
    mj_model, mj_data = build_model_with_MMM(add_whiteboard=False, discard_visuals=True)
    oc.log.write_info(f"Saving model to {mjb_path}")
    mujoco.mj_saveModel(mj_model, mjb_path)

    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    oc.log.start_logging(oc.log.LogLevel.Info)

    # Build full MuJoCo model
    mj_model, mj_data = build_model_with_MMM()
    Ts = mj_model.opt.timestep

    # Controller parameters
    ctr_param = oc.robots.iiwa.IiwaContrParam(simulation=True)
    # Slow and fast normalization factors
    f_c_slow_norm = ctr_param.f_c_slow * (2 * Ts)
    # Joint, cartesian, and gravity compenation parameters
    js_param = oc.robots.robot7.JointCTParameter(ctr_param)
    ts_param = oc.robots.robot7.CartesianCTParameter(ctr_param)
    gc_param = oc.robots.robot7.GravityCompParameter(ctr_param.D_gc)

    iiwa = oc.robots.Iiwa(
        mjb_path, js_param, ts_param, gc_param, oc.Time(Ts), f_c_slow_norm, endeffector_site_name
    )
    param = oc.robots.robot7.HybridForceMotionParameter(simulation=True)
    iiwa.register_HybridForceMotionController(param)

    # Trajectory servers
    com_server_robot = oc.robots.iiwa.TrajectoryServer(iiwa)

    # Calculate inverse dynamics
    def ik_fun(x):
        return inverse_kinematics_residual(
            x,
            mj_model,
            mj_data,
            mj_data.site(endeffector_site_name),
            mj_data.site("starting_point"),
            np.zeros(7),
        )

    bounds = [mj_model.jnt_range[:, 0], mj_model.jnt_range[:, 1]]
    q_starting, _ = mujoco.minimize.least_squares(
        mj_data.qpos, ik_fun, bounds, verbose=1, gtol=1e-7
    )
    mj_data.qpos = q_starting
    mujoco.mj_forward(mj_model, mj_data)
    x_starting = np.zeros(7)
    x_starting[0:3] = mj_data.site(endeffector_site_name).xpos
    mujoco.mju_mat2Quat(x_starting[3:7], mj_data.site(endeffector_site_name).xmat)

    # Preare ready position
    q0 = np.zeros(7)
    iiwa.start(oc.Time(mj_data.time), q_starting, q_starting, oc.Time(1.0))
    # iiwa.set_q_d_nullspace(q0)

    framerate = 60
    frame_time = 1.0 / framerate
    print("frame_time: ", frame_time)
    mj_data.qpos = q_starting

    with mujoco.viewer.launch_passive(mj_model, mj_data) as viewer:
        # Close the viewer automatically after 30 wall-seconds.
        start = time.time()
        last_frame_time = time.time()

        # # Change the camera angle
        viewer.cam.lookat[0:3] = whiteboard_pos
        viewer.cam.lookat[0] += 0.4
        viewer.cam.distance = 2
        viewer.cam.elevation = -15
        viewer.cam.azimuth = -30

        # Variables for path visualization
        contact = []
        trail_idx = 0

        while viewer.is_running():
            # Record the start time of each iteration for timekeeping
            step_start = time.time()

            # Poll trajectory servers for incoming data
            com_server_robot.poll()

            # Update control signals for Robot 0
            iiwa.set_q_act_filtered_derivatives(mj_data.qpos)
            iiwa.robot_data.force_measurement = mj_data.sensordata[0:3]
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
                "force_filtered": iiwa.get_force_filtered().tolist(),
            }
            message = json.dumps(data_dict)
            # Send via UDP
            sock.sendto(message.encode("utf-8"), (UDP_IP, UDP_PORT))

            # Visualize contact trace
            mj_data, viewer, contact, trail_idx = visualize_contact(
                mj_data, viewer, contact, trail_idx
            )

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
