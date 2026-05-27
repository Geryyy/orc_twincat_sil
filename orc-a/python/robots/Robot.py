import socket
import threading
import time
from abc import ABC

import numpy as np
import orcpy.core as oc
import scipy.spatial.transform as sptrans


class Robot(ABC):
    """Base class for all robot types (e.g. LinearAxis, Iiwa)"""

    def __init__(
        self,
        model_path: str,
        local_ip_addr: str,
        robot_ip_addr: str,
        robot_port: int,
        client_port: int,
        udp_timeout: float,
        robot_module,
    ):
        """Initialize object properties such as UDP socket, set time to 0, and
        start the receive thread.

        Args:
            model_path (str): path to MJB file
            local_ip_addr (str): IP address of the local machine. For simulation use 127.0.0.1 (loop-back address)
            robot_ip_addr (str): IP address of the robot. For simulation use 127.0.0.1 (loop-back address)
            robot_port (int): Robot port
            client_port (int): Client port
            udp_timeout (float): UDP timeout
            robot_module (module): Robot type (e.g. orcpy.core.LinearAxis or orcpy.core.Iiwa)
        """
        self.DOF = -1  # overwritten in subclasses
        self.model_path = model_path
        self.module = robot_module
        self.state = robot_module.RobotState()
        self.serializer = self.model.serializer
        self._rx_data = bytearray(self.state.DATA_SIZE)
        self.udp_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self.udp_socket.settimeout(udp_timeout)
        self.udp_socket.setblocking(1)
        self.udp_socket.bind((local_ip_addr, client_port))
        self.robot_address = (robot_ip_addr, robot_port)

        self.time = oc.Time(0)
        # store timing of old trajectory
        self.t1_old = oc.Time(0)

        self._rx_thread = threading.Thread(target=self._recv_thread)
        self._rx_thread.daemon = True
        self._rx_thread.start()
        time.sleep(0.01)  # wait for udp data

    def __del__(self):
        sock = getattr(self, "udp_socket", None)
        if sock is not None:
            sock.close()
        thread = getattr(self, "_rx_thread", None)
        if thread is not None:
            thread.join()

    ### CONVINIENCE FUNCTIONS ###

    def move_jointspace(
        self,
        q1: np.ndarray,
        t0: oc.Time,
        T_traj: oc.Time | float,
        N_pts=10,
        cubic_curve=False,
        velocity_trajectory=False,
        check_if_enabled: bool = False,
        blocking_call: bool = False,
    ):
        """Move robot to given joint configuration at time t0 in T_traj time.

        Args:
          q1 (np.ndarray): [DOFx1] Desired joint configuration
          t0 (arcpy.core.Time): Start time
          T_traj (arcpy.core.Time or float): trajectory duration
          N_pts (int, optional): Number of waypoints. Defaults to 10.
          cubic_curve (bool, optional): Defaults to False.
          velocity_trajectory (bool, optional): Defaults to False.
          blocking_call (bool, optional): Defaults to False. If set True, function call will finish only once the trajectory duration has passed.
        """
        if check_if_enabled and self.state.status == oc.logic.RobotStatus.OFF:
            oc.log.write("INFORMATION: Robot is not enabled")
            return

        self.validate_joint_configuration(q1)

        q0 = self.model.get_q_act().flatten()
        q1 = q1.flatten()
        t0 = max(t0, self.time)
        t1 = t0 + T_traj

        interp = self.module.SplineJointInterpolator(q0, q1, t0, t1)
        interp.init(self.model.get_q_act(), self.model.get_q_dot_act(), np.zeros(self.DOF))

        t_vec = np.linspace(t0.to_sec(), t1.to_sec(), N_pts)
        Joints = []
        Joints_p = []
        t_k = []

        for t in t_vec:
            interp.update(oc.Time(t))
            Joints.append(interp.get_point())
            Joints_p.append(interp.get_derivative())

        # q = q';
        # q_p = q_p';
        # save trajectory timing only if new trajectory
        if t0 >= self.t1_old:
            oc.log.write_info("Move jointspace new trajectory!")
            self.t1_old = t1
            self.t0_old = t0
        else:
            oc.log.write_info("Move jointspace trajectory update!")

        time_k = oc.Time.convert_double_to_time_vector(t_vec)
        if velocity_trajectory:
            self.send_jointspace_velocity_trajectory(time_k, Joints_p)
        else:
            self.send_jointspace_trajectory(time_k, Joints)

        if blocking_call:
            while self.time <= t0 + T_traj:
                time.sleep(0.1)
                pass

        return t_vec, Joints, Joints_p

    def move_taskspace(
        self,
        goal_pose: np.ndarray,
        t0: oc.Time,
        T_traj: oc.Time | float,
        N_pts=10,
        check_if_enabled: bool = False,
        blocking_call: bool = False,
    ):
        """Move Iiwa in task space to the given goal pose from time t0 onwards.

        Args:
            goal_pose (np.ndarray): [4x4] Homogeneous transformation matrix or [7x1] pose vector with the format (x, y, z, qw, qx, qy, qz)
            t0 (arcpy.core.Time): start time
            T_traj (arcpy.core.Time or float): trajectory duration
            N_pts (int, optional): Number of waypoints. Defaults to 10.
            blocking_call (bool, optional): Defaults to False. If set True, function call will finish only once the trajectory duration has passed.
        """
        if check_if_enabled and self.state.status == oc.logic.RobotStatus.OFF:
            oc.log.write_info("INFORMATION: Robot is not enabled")
            return

        pose_start = self.model.get_pose_act()
        quat_start = pose_start[3:7]

        h1_homogenous = True
        if goal_pose.shape[0] == 7:
            h1_homogenous = False  # goal_pose is a pose vector

        if h1_homogenous:
            rot_end = sptrans.Rotation.from_matrix(goal_pose[0:3, 0:3])
            quat_R1 = rot_end.as_quat(scalar_first=True)
        else:
            quat_R1 = goal_pose[3:7]

        # flip end quaternion?
        diff = quat_start - quat_R1
        quat_sum = quat_start + quat_R1
        if diff.dot(diff) < quat_sum.dot(quat_sum):
            quat_end = quat_R1
        else:
            quat_end = -quat_R1

        N = N_pts
        t1 = t0 + T_traj
        t = np.linspace(t0.to_sec(), t1.to_sec(), N, endpoint=True)
        p_start = pose_start[0:3]
        if h1_homogenous:
            p_end = goal_pose[0:3, 3]
        else:
            p_end = goal_pose[0:3]

        posarr = []
        t_k = []

        if t0 < self.t1_old:
            t0 = self.time
        else:
            pass

        r, _ = self._get_path_parameter(t, N)

        rot_array = sptrans.Rotation.from_quat([quat_start, quat_end], scalar_first=True)
        slerp = sptrans.Slerp([0, 1], rot_array)

        for i in np.arange(0, N):
            # rescale r[i] to 0..1
            ri = (r[i] - r[0]) / (r[-1] - r[0])

            rot_ri = slerp(ri)
            quat = rot_ri.as_quat(scalar_first=True)[0]
            p = np.zeros(7)  # x,y,z,qw,qx,qy,qz
            x = np.interp(ri, [0, 1.0], [p_start[0], p_end[0]])
            y = np.interp(ri, [0, 1.0], [p_start[1], p_end[1]])
            z = np.interp(ri, [0, 1.0], [p_start[2], p_end[2]])
            p[0:3] = np.r_[x, y, z]
            p[3:7] = quat
            posarr.append(p)
            t_k.append(t[i])

        # save trajectory timing only if new trajectory
        if t0 >= self.t1_old:
            oc.log.write_info("Move taskspace new trajectory!")
            self.t1_old = t1
            self.t0_old = t0
        else:
            oc.log.write_info("Move taskspace trajectory update!")
        time_k = oc.Time.convert_double_to_time_vector(t_k)
        self.send_taskspace_trajectory(time_k, posarr)

        if blocking_call:
            while self.time <= t0 + T_traj:
                time.sleep(0.1)
                pass

    def set_nullspace(self, t0: oc.Time, q_ns: np.ndarray):
        self.send_nullspace_trajectory(t0, q_ns)
        oc.log.write_info("Set Nullspace!")

    ### SENDING FUNCTIONS ###

    def _send_trajectory(self, data):
        """
        data: bytes or bytearray or list of uint8
        """
        if isinstance(data, bytes | bytearray):
            buffer = data
        else:
            # FlatBuffers returns list of uint8
            buffer = bytes(data)
        self.udp_socket.sendto(buffer, self.robot_address)
        time.sleep(0.001)

    def _send_split_trajectory(self, data_chunks):
        for data in data_chunks:
            self._send_trajectory(data)

    def send_jointspace_trajectory(self, time_pts, joint_pts):
        # Ensure we have Time objects (not floats)
        # TODO adapt time handling if possible for easier access
        time_objs = [t if hasattr(t, "to_sec") else oc.Time(t) for t in time_pts]
        # Joint trajectories can easily exceed the safe UDP payload size when
        # serialized as one FlatBuffer. Send them as split packets so each
        # datagram stays within the receiver's verification limit.
        self._send_split_trajectory(
            self.serializer.serialize_joint_trajectory_split(time_objs, joint_pts)
        )

    def send_taskspace_trajectory(self, time_pts, pose_pts):
        # TODO adapt time handling if possible for easier access
        time_objs = [t if hasattr(t, "to_sec") else oc.Time(t) for t in time_pts]
        self._send_split_trajectory(
            self.serializer.serialize_cartesian_trajectory_split(time_objs, pose_pts)
        )

    def send_cartesian_ctr_parameter_trajectory(self, time, K0, K1, K0N, K1N):
        time_obj = time if hasattr(time, "to_sec") else oc.Time(time)
        param = self.module.CartesianCTParameter(K0, K1, K0N, K1N)
        data = self.serializer.serialize_cartesianctrparam_trajectory(time_obj, param)
        self._send_trajectory(data)

    def send_nullspace_trajectory(self, time, q_ns):
        # TODO adapt time handling if possible for easier access
        time_obj = time if hasattr(time, "to_sec") else oc.Time(time)
        data = self.serializer.serialize_nullspace_trajectory(time_obj, q_ns)
        self._send_trajectory(data)

    def send_dense_jointspace_trajectory(
        self, time_pts, q_pts, q_dot_pts, q_dotdot_pts, tau_ff_pts
    ):
        """Send dense jointspace trajectory with feedforward torques."""
        self.send_dense_jointspace_trajectory_split(
            time_pts, q_pts, q_dot_pts, q_dotdot_pts, tau_ff_pts
        )

    def send_stop(self):
        """Send stop command."""
        data = self.serializer.serialize_stop()
        self._send_trajectory(data)

    def send_cartesian_velocity_trajectory(self, time_pts, velocity_pts):
        time_objs = [t if hasattr(t, "to_sec") else oc.Time(t) for t in time_pts]
        self._send_split_trajectory(
            self.serializer.serialize_cartesian_velocity_trajectory_split(time_objs, velocity_pts)
        )

    def send_jointspace_velocity_trajectory(self, time_pts, velocity_pts):
        time_objs = [t if hasattr(t, "to_sec") else oc.Time(t) for t in time_pts]
        self._send_split_trajectory(
            self.serializer.serialize_jointspace_velocity_trajectory_split(time_objs, velocity_pts)
        )

    def send_joint_ctr_parameter_trajectory(self, time, K0, K1, KI):
        time_obj = time if hasattr(time, "to_sec") else oc.Time(time)
        param = self.module.JointCTParameter(K0, K1, KI)
        data = self.serializer.serialize_jointctrparam_trajectory(time_obj, param)
        self._send_trajectory(data)

    def send_hybrid_force_motion_trajectory(self, pose_pts, force_pts, time_pts):
        time_objs = [t if hasattr(t, "to_sec") else oc.Time(t) for t in time_pts]
        self._send_split_trajectory(
            self.serializer.serialize_hybrid_force_motion_trajectory_split(
                time_objs, pose_pts, force_pts
            )
        )

    def send_dense_jointspace_trajectory_split(
        self, time_pts, q_pts, q_dot_pts, q_dotdot_pts, tau_ff_pts=None
    ):
        """Send a dense (high-frequency) jointspace trajectory with optional feedforward torques.

        This method sends pre-sampled trajectory points that will be executed without
        interpolation on the real-time controller. Use this for true 8kHz trajectories
        where you want exact trajectory execution with feedforward torques.

        Args:
            time_pts: List of oc.Time or array of time stamps [N]
            q_pts: List/array of joint positions [N x DOF]
            q_dot_pts: List/array of joint velocities [N x DOF]
            q_dotdot_pts: List/array of joint accelerations [N x DOF]
            tau_ff_pts: List/array of feedforward torques [N x DOF] (optional, zeros if not provided)
        """
        # Convert arrays to lists of numpy arrays if needed
        if isinstance(q_pts, np.ndarray) and q_pts.ndim == 2:
            q_pts = [q_pts[i, :] for i in range(q_pts.shape[0])]
        if isinstance(q_dot_pts, np.ndarray) and q_dot_pts.ndim == 2:
            q_dot_pts = [q_dot_pts[i, :] for i in range(q_dot_pts.shape[0])]
        if isinstance(q_dotdot_pts, np.ndarray) and q_dotdot_pts.ndim == 2:
            q_dotdot_pts = [q_dotdot_pts[i, :] for i in range(q_dotdot_pts.shape[0])]

        # Handle optional feedforward torques
        if tau_ff_pts is None:
            tau_ff_pts = [np.zeros(self.DOF) for _ in range(len(time_pts))]
        elif isinstance(tau_ff_pts, np.ndarray) and tau_ff_pts.ndim == 2:
            tau_ff_pts = [tau_ff_pts[i, :] for i in range(tau_ff_pts.shape[0])]

        # Ensure Time objects (not floats)
        time_objs = [t if hasattr(t, "to_sec") else oc.Time(t) for t in time_pts]

        # Serialize and send using FlatBuffers
        self._send_split_trajectory(
            self.serializer.serialize_dense_joint_trajectory_split(
                time_objs, q_pts, q_dot_pts, q_dotdot_pts, tau_ff_pts
            )
        )

    ### UTILITY FUNCTIONS ###

    def _recv_thread(self):
        try:
            while True:
                nbytes, self_sender = self.udp_socket.recvfrom_into(
                    self._rx_data, self.module.RobotState.DATA_SIZE
                )

                state = self.module.RobotState.deserialize(bytearray(self._rx_data[:nbytes]))
                self.state = state
                self.time = state.time

                # Write received state into model and update
                self.model.set_q_act(state.q_act)
                self.model.set_q_dot_act(state.q_dot_act)
                self.model.set_q_dotdot_act(state.q_dotdot_act)
                self.model.update(state.time)

                self.robot_address = self_sender
        except OSError as os_error:
            # Handle specific OSError
            oc.log.write_info(f"Error in _recv_thread: {os_error}")
        finally:
            oc.log.write_info("Exiting _recv_thread")

    def _get_path_parameter(self, t_vec: np.ndarray, N: int, cubic_curve=False):
        t0 = t_vec[0]
        t1 = t_vec[-1]
        r = np.zeros((N, 1))
        v = np.zeros((N, 1))

        for i in np.arange(0, N):
            ti = (t_vec[i] - t0) / (t1 - t0)
            # ti = i /float(N-1) # i = 0 .. (N-1)

            # normalized cubic polynomial as pathparameter
            # provides a trajectory that can be continuously differentiated twice
            if cubic_curve:
                r[i] = -2 * ti**3 + 3 * ti**2  # cubic
            else:
                hard_lin = False
                if hard_lin:
                    r[i] = ti  # linear, velocity jump at start and stop!
                    continue
                # smooth start and stop
                # for more details see: to 2021_03_14_Trajektorienplanung.mw
                # x in (0, 0.5), acceleration and deacceleration fraction of trajectory, in between constant velocity
                x = 0.2
                A = -1 / x / (-1 + x)
                # path parameter
                if ti <= 0:
                    r[i] = 0
                elif ti <= x:
                    r[i] = A / 2 * ti**2
                elif ti <= 1 - x:
                    r[i] = A / 2 * x * (2 * ti - x)
                elif ti <= 1:
                    r[i] = A / 2 * (-(ti**2) - 2 * x**2 + 2 * ti + 2 * x - 1)
                elif ti > 1:
                    r[i] = A / 2 * -2 * x * (-1 + x)
                else:
                    r[i] = 0

                # first derivative of path parameter
                if ti <= 0:
                    v[i] = 0
                elif ti <= x:
                    v[i] = A * ti
                elif ti <= 1 - x:
                    v[i] = A * x
                elif ti <= 1:
                    v[i] = A * (1 - ti)
                elif ti > 1:
                    v[i] = 0
                else:
                    v[i] = 0

        return r, v

    def check_robot_model(self):
        """Check if model identifiers from external RobotState and the model itself match"""
        if self.state.model_id != self.model.get_model_id():
            oc.log.write_info("ERROR: Model identifier on client and server differ!")
            exit()

    def change_robot_model(self, model_path: str):
        """Change robot model to MJB at model_path

        Args:
            model_path (str): path to MJB file
        """
        self.model_path = model_path
        self.model = oc.robots.Iiwa(model_path)
        self.check_robot_model()

    def validate_joint_configuration(self, array):
        """Checks array type to be np.ndarray and its shape to be a valid joint configuration.

        Args:
            array (np.ndarray): Input array to be checked.

        Raises:
            TypeError
            ValueError
        """
        if not isinstance(array, np.ndarray):
            raise TypeError(f"Input must be of type np.ndarray, but got {type(array).__name__}.")
        if array.shape not in ((self.DOF,), (self.DOF, 1)):
            raise ValueError(
                f"Input array must have shape ({self.DOF},) or ({self.DOF},1), but got {array.shape}."
            )
