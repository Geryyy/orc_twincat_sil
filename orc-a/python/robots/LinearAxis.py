# orcpy/robots/LinearAxis.py
import warnings

import orcpy.core as oc

from .Robot import Robot
from .util_functions import default_model_path


class LinearAxis(Robot):
    """Communication with linear axis"""

    def __init__(
        self,
        model_path: str = "models/linear_axis.mjb",
        local_ip_addr: str = "127.0.0.1",
        robot_ip_addr: str = "127.0.0.1",
        server_port: int = oc.robots.LinearAxis.SERVER_PORT,
        client_port: int = oc.robots.LinearAxis.CLIENT_PORT,
        udp_timeout: float = 1.0,
        robot_port: int | None = None,
    ):
        """Create LinearAxis object.

        Args:
            model_path (str): path to MJB file
            local_ip_addr (str): IP address of the local machine.
            robot_ip_addr (str): IP address of the robot.
            server_port (int): Robot (server) port. Renamed from ``robot_port``
                in 0.3.0 for API parity with Iiwa/Kinova.
            client_port (int): Client port.
            udp_timeout (float): UDP timeout in seconds.
            robot_port (int, optional): Deprecated alias for ``server_port``.
        """
        if robot_port is not None:
            warnings.warn(
                "LinearAxis(robot_port=...) is deprecated, use "
                "server_port=... for API parity with Iiwa/Kinova.",
                DeprecationWarning,
                stacklevel=2,
            )
            server_port = robot_port
        self.model = oc.robots.LinearAxis(model_path)
        super().__init__(
            model_path,
            local_ip_addr,
            robot_ip_addr,
            server_port,
            client_port,
            udp_timeout,
            oc.robots.robot2,
        )
        self.DOF = 2

    @classmethod
    def from_config(cls, config: dict):
        """Create LinearAxis object from configuration dictionary.

        Args:
            config (dict): Configuration dictionary containing the keys 'model_path',
            'local_ip_addr', 'robot_ip_addr', 'robot_port' and 'client_port'.

        Returns:
            LinearAxis: LinearAxis object.
        """
        return cls(
            config["model_path"],
            config["local_ip_addr"],
            config["robot_ip_addr"],
            config["robot_port"],
            config["client_port"],
        )

    @classmethod
    def from_default_config(cls, simulation=False):
        """Create LinearAxis object from default configuration.

        Args:
            simulation (bool, optional): Simulation flag. Defaults to False.

        Returns:
            LinearAxis: LinearAxis object.
        """
        if simulation:
            return cls(
                default_model_path("linear_axis.mjb"),
                "127.0.0.1",
                "127.0.0.1",
                40002,
                41002,
                udp_timeout=1.0,
            )

        return cls(
            default_model_path("linear_axis.mjb"),
            "192.168.2.3",
            "192.168.2.10",
            oc.robots.LinearAxis.SERVER_PORT,
            oc.robots.LinearAxis.CLIENT_PORT,
            udp_timeout=1.0,
        )
