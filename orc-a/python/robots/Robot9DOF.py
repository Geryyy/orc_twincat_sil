import copy

from .Iiwa import Iiwa
from .LinearAxis import LinearAxis
from .util_functions import default_model_path

SIMULATION_IP_ADDR = "127.0.0.1"
SIMULATION_IIWA_ROBOT_PORT = 20001
SIMULATION_IIWA_CLIENT_PORT = 21001
SIMULATION_LAXIS_ROBOT_PORT = 40002
SIMULATION_LAXIS_CLIENT_PORT = 41002

ROBOT_LOCAL_IP_ADDR = "192.168.2.3"
ROBOT_ROBOT_IP_ADDR = "192.168.2.10"
ROBOT_IIWA_ROBOT_PORT = 10000
ROBOT_IIWA_CLIENT_PORT = 11000
ROBOT_LAXIS_ROBOT_PORT = 10010
ROBOT_LAXIS_CLIENT_PORT = 11010
ROBOT_IIWA_ENDEFFECTOR_SITE_NAME = "iiwa_link_e"

IIWA_MODEL_PATH = default_model_path("iiwa_hanging.mjb")
LAXIS_MODEL_PATH = default_model_path("linear_axis.mjb")

CONFIG_SIMULATION = {
    "local_ip_addr": SIMULATION_IP_ADDR,
    "robot_ip_addr": SIMULATION_IP_ADDR,
    "iiwa": {
        "robot_port": SIMULATION_IIWA_ROBOT_PORT,
        "client_port": SIMULATION_IIWA_CLIENT_PORT,
        "model_path": IIWA_MODEL_PATH,
        "endeffector_site_name": ROBOT_IIWA_ENDEFFECTOR_SITE_NAME,
    },
    "laxis": {
        "robot_port": SIMULATION_LAXIS_ROBOT_PORT,
        "client_port": SIMULATION_LAXIS_CLIENT_PORT,
        "model_path": LAXIS_MODEL_PATH,
    },
}

CONFIG_ROBOT = {
    "local_ip_addr": ROBOT_LOCAL_IP_ADDR,
    "robot_ip_addr": ROBOT_ROBOT_IP_ADDR,
    "iiwa": {
        "robot_port": ROBOT_IIWA_ROBOT_PORT,
        "client_port": ROBOT_IIWA_CLIENT_PORT,
        "model_path": IIWA_MODEL_PATH,
        "endeffector_site_name": ROBOT_IIWA_ENDEFFECTOR_SITE_NAME,
    },
    "laxis": {
        "robot_port": ROBOT_LAXIS_ROBOT_PORT,
        "client_port": ROBOT_LAXIS_CLIENT_PORT,
        "model_path": LAXIS_MODEL_PATH,
    },
}


class Robot9DOF:
    def __init__(
        self,
        local_ip_addr: str = None,
        robot_ip_addr: str = None,
        laxis: dict = None,
        iiwa: dict = None,
    ):
        """Initializes a 9 DOF robot consisting of one linear axis and one Iiwa
        robot.

        Args:
            local_ip_addr (str, optional): Local IP address. Defaults to None.
            robot_ip_addr (str, optional): Robot IP address. Defaults to None.
            laxis (dict, optional): Linear axis configuration struct with keys: 'model_path', 'robot_port', 'client_port'. Defaults to None.
            iiwa (dict, optional): Iiwa robot configuration struct with keys: 'model_path', 'robot_port', 'client_port', 'endeffector_site_name'. Defaults to None.
        """

        self.laxis = LinearAxis(
            laxis["model_path"],
            local_ip_addr,
            robot_ip_addr,
            laxis["robot_port"],
            laxis["client_port"],
        )
        self.iiwa = Iiwa(
            iiwa["model_path"],
            local_ip_addr,
            robot_ip_addr,
            iiwa["robot_port"],
            iiwa["client_port"],
            endeffector_site_name=iiwa["endeffector_site_name"],
        )

    @classmethod
    def get_default_config(cls, simulation: bool = False):
        if simulation:
            return copy.deepcopy(CONFIG_SIMULATION)
        else:
            return copy.deepcopy(CONFIG_ROBOT)

    @classmethod
    def from_default_config(cls, simulation: bool = False):
        """Initializes a 9 DOF robot using the default configuration dictionary.

        Args:
            simulation (bool, optional): Simulation flag. Defaults to False.
        Returns:
            Robot9DOF: A Robot9DOF object.
        """
        return cls(**cls.get_default_config(simulation))
