import numpy as np
import orcpy.core as oc
import orcpy.robots.util_functions as util_functions

from .Robot import Robot


class Kinova(Robot):
    is_infinite = [True, False, True, False, True, False, True]

    """Communication with Kinova"""

    def __init__(
        self,
        model_path: str = "models/kinova3.mjb",
        local_ip_addr: str = "127.0.0.1",
        robot_ip_addr: str = "127.0.0.1",
        server_port: int = oc.robots.kinova.SERVER_PORT,
        client_port: int = oc.robots.kinova.CLIENT_PORT,
        udp_timeout: float = 1.0,
    ) -> None:
        """Create Kinova object.

        Args:
            model_path (str): path to MJB file
            local_ip_addr (str): IP address of the local machine. For simulation use 127.0.0.1 (loop-back address)
            robot_ip_addr (str): IP address of the robot. For simulation use 127.0.0.1 (loop-back address)
            server_port (int): Robot port
            client_port (int): Client port
            udp_timeout (float): UDP timeout
        """

        self.model = oc.robots.Kinova(model_path)
        super().__init__(
            model_path,
            local_ip_addr,
            robot_ip_addr,
            server_port,
            client_port,
            udp_timeout,
            oc.robots.robot7,
        )
        self.DOF = 7  # degrees of freedom

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
        if self.check_if_valid_configuration(q1):
            # Wrap destination to allow shortest path for infinite joints
            q1[Kinova.is_infinite] = util_functions.wrap_to_pi(q1[Kinova.is_infinite])
            q1 = self.wrap_desired_q_to_shortest_path(self.model.get_q_act(), q1)
            super().move_jointspace(
                q1,
                t0,
                T_traj,
                N_pts,
                cubic_curve,
                velocity_trajectory,
                check_if_enabled,
                blocking_call,
            )
        else:
            oc.log.write_error("Invalid joint configuration, not sending command to robot.")
            pass

    def check_if_valid_configuration(self, q: np.ndarray) -> bool:
        """Check if the given joint configuration is valid (within joint limits). Although from Kinova's datasheet the joint limits
        are centered around 0, here they get wrapped into the range [0, 360]°. The limits are as follows:
        - Joint 2: [-128.9, 128.9] degrees
        - Joint 4: [-147.8, 147.8] degrees
        - Joint 6: [-120.3, 120.3] degrees

        Taken from Kinova Gen3 user guide: https://www.kinovarobotics.com/uploads/User-Guide-Gen3-R07.pdf

        Args:
            q (np.ndarray): Joint configuration to check
        """

        if q.size != self.DOF:
            oc.log.write_error(
                f"Invalid joint configuration size: expected {self.DOF}, got {q.size}"
            )
            return False

        q = np.rad2deg(q)  # Convert to degrees for limit checking
        q = q % 360
        if not (0 <= q[1] < 128.9 or 331.1 < q[1] <= 360):
            oc.log.write_error(f"Joint 2 out of limits: {q[1]:.2f} degrees")
            return False
        if not (0 <= q[3] < 147.8 or 212.2 < q[3] <= 360):
            oc.log.write_error(f"Joint 4 out of limits: {q[3]:.2f} degrees")
            return False
        if not (0 <= q[5] < 120.3 or 240.3 < q[5] <= 360):
            oc.log.write_error(f"Joint 6 out of limits: {q[5]:.2f} degrees")
            return False

        return True

    @staticmethod
    def wrap_desired_q_to_shortest_path(q_current: np.ndarray, q_desired: np.ndarray) -> np.ndarray:
        """Wrap the desired joint configuration of infinite joints to the shortest path from the current configuration.

        Args:
            q_current (np.ndarray): Current joint configuration in radians
            q_desired (np.ndarray): Desired joint configuration in radians

        Returns:
            np.ndarray: Wrapped desired joint configuration in radians
        """
        q_diff = q_desired - q_current
        q_diff[Kinova.is_infinite] = (q_diff[Kinova.is_infinite] + np.pi) % (2 * np.pi) - np.pi
        return q_current + q_diff

    @staticmethod
    def get_joint_limits():
        min_ranges = np.deg2rad(
            np.array([-np.inf, -128.0, -np.inf, -147.8, -np.inf, -120.3, -np.inf])
        )
        max_ranges = np.deg2rad(np.array([np.inf, 128.0, np.inf, 147.8, np.inf, 120.3, np.inf]))
        return [min_ranges, max_ranges]
