import numpy as np
import orcpy.core as oc

from .Robot import Robot
from .util_functions import default_model_path


class Iiwa(Robot):
    """Communication with iiwa"""

    def __init__(
        self,
        model_path: str = "models/iiwa_standing.mjb",
        local_ip_addr: str = "127.0.0.1",
        robot_ip_addr: str = "127.0.0.1",
        server_port: int = oc.robots.iiwa.SERVER_PORT,
        client_port: int = oc.robots.iiwa.CLIENT_PORT,
        udp_timeout: float = 1.0,
        endeffector_site_name: str = None,
    ) -> None:
        """Create Iiwa object.

        Args:
            model_path (str): path to MJB file
            local_ip_addr (str): IP address of the local machine. For simulation use 127.0.0.1 (loop-back address)
            robot_ip_addr (str): IP address of the robot. For simulation use 127.0.0.1 (loop-back address)
            server_port (int): Robot port
            client_port (int): Client port
            udp_timeout (float): UDP timeout
        """
        if endeffector_site_name is None:
            self.model = oc.robots.Iiwa(model_path)
        else:
            self.model = oc.robots.Iiwa(model_path, endeffector_site_name)

        super().__init__(
            model_path,
            local_ip_addr,
            robot_ip_addr,
            server_port,
            client_port,
            udp_timeout,
            oc.robots.robot7,
        )
        # if local_ip_addr!='127.0.0.1': # check only on LabRobot
        #     self.check_robot_model()
        self.DOF = 7  # degrees of freedom

    @classmethod
    def from_config(cls, config: dict):
        """Create Iiwa object from custom configuration dictionary.

        Args:
            config (dict): Configuration dictionary containing the keys 'model_path',
            'local_ip_addr', 'robot_ip_addr', 'robot_port' and 'client_port'.

        Returns:
            Iiwa: Iiwa object
        """
        return cls(
            config["model_path"],
            config["local_ip_addr"],
            config["robot_ip_addr"],
            config["robot_port"],
            config["client_port"],
        )

    @classmethod
    def from_default_config(cls, simulation: bool = False):
        """Create Iiwa object from default configuration.

        Args:
            simulation (bool, optional): Set to true if Mujoco simulation is wanted. Defaults to False.

        Returns:
            Iiwa: Iiwa object
        """

        if simulation:
            return cls(
                model_path=default_model_path("iiwa_standing.mjb"),
                local_ip_addr="127.0.0.1",
                robot_ip_addr="127.0.0.1",
                server_port=oc.robots.iiwa.SERVER_PORT,
                client_port=oc.robots.iiwa.CLIENT_PORT,
                udp_timeout=1.0,
            )

        return cls(
            model_path=default_model_path("iiwa_standing.mjb"),
            local_ip_addr="192.168.1.3",
            robot_ip_addr="192.168.1.10",
            server_port=oc.robots.iiwa.SERVER_PORT,
            client_port=oc.robots.iiwa.CLIENT_PORT,
            udp_timeout=1.0,
        )

    def forward_kinematics(self, q: np.ndarray) -> np.ndarray:
        """Forward kinematics of Iiwa Robot

        Args:
          q (ndarray): [7x1] Vector

        Returns:
          [4x4] Matrix: Homogeneous transformation matrix
        """
        model = self.model.copy()
        model.set_q_act(q)
        model.update(oc.Time(0.0))
        return model.get_current_H_0_e()

    # def move_jointspace_jerk(self, time_points: np.ndarray, jerk_matrix: np.ndarray, check_if_enabled : bool = True):
    #     """Move the robot based on the jerk input at the specified times. The
    #     trajectory is calculated by integrating linear jerk functions.

    #     Args:
    #         time_points ([tx1] np.ndarray]): Time points at which the jerk values are specified
    #         jerk_matrix ([jxt] np.ndarray]): Jerk points for every time step and every axis
    #     """
    #     if check_if_enabled and self.state.status==oc.logic.RobotStatus.OFF:
    #         print("INFORMATION: Robot is not enabled")
    #         return

    #     if time_points[0] >= self.t1_old:
    #         print("Move jointspace jerk new trajectory!")
    #         self.t1_old = time_points[-1]
    #         self.t0_old = time_points[0]
    #     else:
    #         print("Move jointspace jerk trajectory update!")

    #     self.send_jointspace_jerk_trajectory(
    #         time_points.tolist(), jerk_matrix.T.tolist())

    # def move_taskspace_jerk(self, time_points, jerk_matrix):
    #     """Move the robot based on the jerk input at the specified times. The
    #     trajectory is calculated by integrating linear jerk functions.

    #     Args:
    #         time_points (): array t x 1 Time points at which the jerk values are specified
    #         jerk_matrix (): matrix 6 x t Jerk points for every time step and every spacial dimension
    #     """
    #     if time_points[0] >= self.t1_old:
    #         print("Move jointspace jerk new trajectory!")
    #         self.t1_old = time_points[-1]
    #         self.t0_old = time_points[0]
    #     else:
    #         print("Move jointspace jerk trajectory update!")

    #     self.send_taskspace_jerk_trajectory(
    #         time_points.tolist(), jerk_matrix.T.tolist())
