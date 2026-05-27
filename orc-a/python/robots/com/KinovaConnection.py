import numpy as np

try:
    from kortex_api.autogen.client_stubs.ActuatorConfigClientRpc import ActuatorConfigClient
    from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
    from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
    from kortex_api.autogen.client_stubs.DeviceManagerClientRpc import DeviceManagerClient
    from kortex_api.autogen.messages import (
        ActuatorConfig_pb2,
        Base_pb2,
        BaseCyclic_pb2,
        Common_pb2,
        Session_pb2,
    )
    from kortex_api.RouterClient import RouterClient, RouterClientSendOptions
    from kortex_api.SessionManager import SessionManager
    from kortex_api.TCPTransport import TCPTransport
    from kortex_api.UDPTransport import UDPTransport
except ModuleNotFoundError:
    import sys

    sys.exit(
        "Kortex API for Kinova not intalled. Run:\n\n\tpip install third_party/kinova-kortex-api/kortex_api-2.6.0.post3-py3-none-any.whl\n\nto install the library and rerun this script."
    )


import logging
import socket
import sys
import threading
import time

import orcpy.core as oc
import orcpy.robots.util_functions as util_functions
import orjson

# Add ms to logging
logger = logging.getLogger(__name__)
logging.basicConfig(
    format="%(asctime)s.%(msecs)03d - %(levelname)s: %(message)s",
    datefmt="%H:%M:%S",
    level=logging.INFO,
)

TIMEOUT_DURATION = 20


class KinovaConnection:
    def __init__(self):
        self.DOF = 7

        self.ROBOT_IP = "192.168.131.40"
        self.TCP_PORT = 10000
        self.UDP_PORT = 10001
        self.failed_cyclic_count = 0

        self.n = 7

        self.dt = 1e-3
        self.t0 = oc.Time(time.perf_counter())
        self.t = oc.Time(time.perf_counter())
        self.state = None
        self.robot = None

        self.sendOption = RouterClientSendOptions()
        self.sendOption.andForget = False
        self.sendOption.delay_ms = 0
        self.sendOption.timeout_ms = 3  # This was 50 ms

        self.mode = "Idle"

        self._host = "127.0.0.1"
        self._port = 9870
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.stop_event = threading.Event()

        self.tau_sens = None

        self.voltage_series = np.empty((0, 1))
        self.current_series = np.empty((0, 1))
        self.torque_series = np.empty((0, 7))
        self.q_pos_series = np.empty((0, 7))

        self.emergency_mode = False  # Emergency mode is activated, if a collision is detected

    def send(self, data: dict):
        try:
            self._sock.sendto(orjson.dumps(data), (self._host, self._port))
        except Exception as e:
            logger.error(f"Failed to send data: {e}")

    def __enter__(self):
        # TCP Connection
        self.TCP_transport = TCPTransport()
        self.TCP_router = RouterClient(self.TCP_transport, RouterClient.basicErrorCallback)
        self.TCP_transport.connect(self.ROBOT_IP, self.TCP_PORT)

        # UDP Connection (Realtime)
        self.UDP_transport = UDPTransport()
        self.UDP_router = RouterClient(self.UDP_transport, RouterClient.basicErrorCallback)
        self.UDP_transport.connect(self.ROBOT_IP, self.UDP_PORT)

        # Session Manager
        session_info = Session_pb2.CreateSessionInfo()
        session_info.username = "admin"
        session_info.password = "admin"
        session_info.session_inactivity_timeout = 60000
        session_info.connection_inactivity_timeout = 2000

        # TCP Session Manager
        self.TCP_session_manager = SessionManager(self.TCP_router)
        self.TCP_session_manager.CreateSession(session_info)

        # UDP Session Manager
        self.UDP_session_manager = SessionManager(self.UDP_router)
        self.UDP_session_manager.CreateSession(session_info)

        # Device Manager
        self.device_manager = DeviceManagerClient(self.TCP_router)

        # Services / Base
        # TCP
        self.TCP_base = BaseClient(self.TCP_router)
        # UDP
        self.UDP_base = BaseCyclicClient(self.UDP_router)

        # Actuator Config
        self.TCP_actuator = ActuatorConfigClient(self.TCP_router)

        # Init Basic Command
        self.command = BaseCyclic_pb2.Command()
        device_handles = self.device_manager.ReadAllDevices().device_handle

        for handle in device_handles:
            if (
                handle.device_type == Common_pb2.BIG_ACTUATOR
                or handle.device_type == Common_pb2.SMALL_ACTUATOR
            ):
                self.command.actuators.add()

        # Time Init
        self.t = oc.Time(time.perf_counter())
        self.t_step = self.t

        # State Information Thread
        self.update_thread = threading.Thread(target=self.update)
        self.update_thread.daemon = True
        self.update_thread.start()

        return self

    def __exit__(self, exc_type, exc_value, traceback):
        logger.warning(f"{exc_type}")
        logger.warning(f"{exc_value}")
        logger.warning(f"{traceback}")

        servoing_mode = Base_pb2.ServoingModeInformation()
        servoing_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
        self.TCP_base.SetServoingMode(servoing_mode)

        # Set Robot into Position Mode
        control_mode = ActuatorConfig_pb2.ControlModeInformation()
        control_mode.control_mode = ActuatorConfig_pb2.ControlMode.Value("POSITION")
        for i in range(self.DOF):
            self.TCP_actuator.SetControlMode(control_mode, i + 1)

        logger.info("Single Level Servoing - Position Mode - Set")

        # Close Update Thread
        if self.update_thread is not None:
            self.stop_event.set()
            self.update_thread.join(timeout=5)
            logger.info("Control Thread Stopped")

        # Close Session manager
        if self.TCP_session_manager is not None:
            self.TCP_session_manager.CloseSession()
            logger.debug("TCP Session closed")

        if self.UDP_session_manager is not None:
            self.UDP_session_manager.CloseSession()
            logger.debug("UDP Session closed")

        # Disconnect Router
        if self.TCP_router is not None:
            self.TCP_router.SetActivationStatus(False)
            logger.debug("TCP Router disabled")

        if self.UDP_router is not None:
            self.UDP_router.SetActivationStatus(False)
            logger.debug("UDP Router disabled")

        # Disconnect Transport
        if self.TCP_transport is not None:
            self.TCP_transport.disconnect()
            logger.debug("TCP Transport disconnected")

        if self.UDP_transport is not None:
            self.UDP_transport.disconnect()
            logger.debug("UDP Transport disconnected")

        logger.info("Hardware connection closed")

    def check_for_end_or_abort(self, e):
        def check(notification, e=e):
            logger.info("EVENT : " + Base_pb2.ActionEvent.Name(notification.action_event))
            if (
                notification.action_event == Base_pb2.ACTION_END
                or notification.action_event == Base_pb2.ACTION_ABORT
            ):
                e.set()

        return check

    def set_robot(self, robot):
        self.robot = robot
        self.com_server_robot = oc.robots.kinova.TrajectoryServer(self.robot)
        time.sleep(0.5)

    def get_state(self):
        self.state = self.UDP_base.RefreshFeedback()
        return self.state

    def update(self):
        while not self.stop_event.is_set():
            self.t = oc.Time(time.perf_counter())
            if self.t >= self.t_step + self.dt:
                self.state = self.UDP_base.RefreshFeedback()

                # Write measurement to model
                # Transform angle to be in range [-pi, pi]
                q_wrapped = util_functions.wrap_to_pi(
                    np.deg2rad(np.array([act.position for act in self.state.actuators]))
                )
                # Unwrap the angle
                q_unwrapped = q_wrapped
                q_prior = self.robot.get_q_act()
                for i in range(len(q_unwrapped)):
                    q_unwrapped[i] = np.unwrap([q_prior[i], q_wrapped[i]])[1]
                qp = np.deg2rad(np.array([act.velocity for act in self.state.actuators]))

                self.robot.set_q_act(q_unwrapped)
                self.robot.set_q_dot_act(qp)

                if not self.emergency_mode:
                    # Poll for incoming trajectories
                    self.com_server_robot.poll()

                if self.mode == "TORQUE":
                    self.apply_torque()

                print(self.robot.get_current_H_0_e()[:3, -1])

                # Send to PlotJuggler for viewing
                state_dict = {
                    "q_act": self.robot.get_q_act().tolist(),
                    "q_set": self.robot.get_q_set().tolist(),
                    "tau_act": self.robot.get_tau_act().tolist(),
                    "e_js_robot": self.robot.get_joint_error().tolist(),
                    "e_ts_robot": self.robot.get_cartesian_error().tolist(),
                    "pose_act": self.robot.get_pose_act().tolist(),
                    "pose_set": self.robot.get_pose_set().tolist(),
                    "x_dot_act": self.robot.get_x_dot_act().tolist(),
                    "x_dot_set": self.robot.get_x_dot_set().tolist(),
                    "dt": (self.t - self.t_step).to_sec(),
                    "q_raw": np.deg2rad(
                        np.array([act.position for act in self.state.actuators])
                    ).tolist(),
                    "q_dot_act": qp.tolist(),
                    "q_dot_d": self.robot.get_q_dot_set().tolist(),
                    "q_dotdot_d": self.robot.get_q_dotdot_set().tolist(),
                }
                self.send(state_dict)
                self.t_step = self.t

                # Send robot state via UDP
                self.com_server_robot.send_robot_data(self.t)

    def set_torque_mode(self):
        if self.robot is None:
            logger.error("No robot set")

        # Set Actuators into Servoing Mode
        for i in range(self.DOF):
            self.command.actuators[i].flags = 1
            self.command.actuators[i].position = self.state.actuators[i].position
            self.command.actuators[i].torque_joint = -self.state.actuators[i].torque

        logger.info("Inital Command applied")

        # Set Torque Mode (Low level servoing)
        servoing_mode = Base_pb2.ServoingModeInformation()
        servoing_mode.servoing_mode = Base_pb2.LOW_LEVEL_SERVOING
        self.TCP_base.SetServoingMode(servoing_mode)
        logger.info("Low level servoing set")

        self.UDP_base.Refresh(self.command)
        logger.info("Initial Command sent")

        control_mode = ActuatorConfig_pb2.ControlModeInformation()
        control_mode.control_mode = ActuatorConfig_pb2.ControlMode.Value("TORQUE")
        for i in range(self.DOF):
            self.TCP_actuator.SetControlMode(control_mode, i + 1)
        logger.info("Torque mode set")

        self.mode = "TORQUE"

    def apply_torque(self):
        if self.robot.robot_data.collision_detected and not self.emergency_mode:
            # Enter emergency mode and send stop trajectory
            self.emergency_mode = True
            oc.log.write_warning(
                "Emergency mode activated due to collision, trajectory server is not active!"
            )
            self.robot.add_jointspace_trajectory(
                self.robot.get_q_act(),
                self.robot.get_q_act(),
                self.robot.robot_data.t,
                self.robot.robot_data.t + 0.5,
            )

        self.robot.update(self.t)
        tau = self.robot.get_tau_act()

        # Set Actuators Command
        for i in range(self.DOF):
            self.command.actuators[i].torque_joint = tau[i]
            self.command.actuators[i].position = np.rad2deg(
                util_functions.wrap_to_2pi(self.robot.get_q_set()[i])
            )

        self.command.frame_id += 1
        if self.command.frame_id > 65535:
            self.command.frame_id = 0
        for i in range(self.DOF):
            self.command.actuators[i].command_id = self.command.frame_id

        try:
            self.UDP_base.Refresh(self.command, 0, self.sendOption)
        except:
            self.failed_cyclic_count = self.failed_cyclic_count + 1
            logger.debug(f"failed cycle {self.failed_cyclic_count}")

    def goto(self, name="Home"):
        # Set Servoing Mode to single level servoing
        servoing_mode = Base_pb2.ServoingModeInformation()
        servoing_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
        self.TCP_base.SetServoingMode(servoing_mode)

        # Move to home position
        action_type = Base_pb2.RequestedActionType()
        action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
        action_list = self.TCP_base.ReadAllActions(action_type)
        action_handle = None
        for action in action_list.action_list:
            if action.name == name:
                action_handle = action.handle

        if action_handle is None:
            logger.warning(
                f"Invalid home position name: '{name}' <-> ['Home','Retract','Packaging','Zero']"
            )
            sys.exit(0)

        e = threading.Event()
        notification_handle = self.TCP_base.OnNotificationActionTopic(
            self.check_for_end_or_abort(e), Base_pb2.NotificationOptions()
        )

        # Execute Action
        self.TCP_base.ExecuteActionFromReference(action_handle)

        finished = e.wait(TIMEOUT_DURATION)
        self.TCP_base.Unsubscribe(notification_handle)

        if finished:
            logger.info(f"'{name}' Position reached")
        else:
            logger.error("Timeout")
        return finished
