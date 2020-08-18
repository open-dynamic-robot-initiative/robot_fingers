"""Classes and functions to easily set up robot demo scripts."""
import os
import rospkg

import robot_interfaces
import robot_fingers


#: Default configurations for various robots.
#: Maps robot names to a tuple of
#:  1) the module defining the corresponding types for this robot,
#:  2) a function to create the backend, and
#:  3) the name of the configuration file
#:
#: This corresponds to the arguments of the :class:`Robot` class.
robot_configs = {
    "fingerone": (
        robot_interfaces.finger,
        robot_fingers.create_real_finger_backend,
        "finger.yml",
    ),
    "trifingerone": (
        robot_interfaces.trifinger,
        robot_fingers.create_trifinger_backend,
        "trifinger.yml",
    ),
    "fingeredu": (
        robot_interfaces.finger,
        robot_fingers.create_real_finger_backend,
        "fingeredu.yml",
    ),
    "trifingeredu": (
        robot_interfaces.trifinger,
        robot_fingers.create_trifinger_backend,
        "trifingeredu.yml",
    ),
    "trifingerpro": (
        robot_interfaces.trifinger,
        robot_fingers.create_trifinger_backend,
        "trifingerpro.yml",
    ),
    "trifingerpro_calib": (
        robot_interfaces.trifinger,
        robot_fingers.create_trifinger_backend,
        "trifingerpro_for_calib.yml",
    ),
    "onejoint": (
        robot_interfaces.one_joint,
        robot_fingers.create_one_joint_backend,
        "onejoint.yml",
    ),
    "twojoint": (
        robot_interfaces.two_joint,
        robot_fingers.create_two_joint_backend,
        "twojoint.yml",
    ),
}


class Robot:

    @classmethod
    def create_by_name(cls, robot_name):
        """Create a ``Robot`` instance for the specified robot.

        Args:
            robot_name (str):  Name of the robots.  See
            :meth:`Robot.get_supported_robots` for a list of supported robots.

        Returns:
            Robot: A ``Robot`` instance for the specified robot.
        """
        return cls(*robot_configs[robot_name])

    @staticmethod
    def get_supported_robots():
        """Get list of robots supported by ``create_by_name``.

        Returns:
            List of supported robot names.
        """
        return robot_configs.keys()

    def __init__(
        self, robot_module, create_backend_function, config_file_name
    ):
        """Initialize the robot environment (backend and frontend).

        :param robot_module: The module that defines the robot classes (i.e.
            `Data`, `Frontend`, `Backend`, ...)
        :param create_backend_function: Function to create the robot backend.
        :param config_file_name: Either an absolute path to a config file
            (needs to start with "/" in this case) or the name of a config
            file located in robot_fingers/config.
        """
        # convenience mapping of the Action type
        self.Action = robot_module.Action

        # If config_file_name starts with "/" assume it is an absolute path and
        # use it as is.  Otherwise interpret it relative to the config
        # directory of the robot_fingers package.
        if config_file_name.startswith("/"):
            config_file_path = config_file_name
        else:
            config_file_path = os.path.join(
                rospkg.RosPack().get_path("robot_fingers"),
                "config",
                config_file_name,
            )

        # Storage for all observations, actions, etc.
        self.robot_data = robot_module.SingleProcessData()

        # The backend sends actions from the data to the robot and writes
        # observations from the robot to the data.
        self.backend = create_backend_function(
            self.robot_data, config_file_path
        )

        #: The frontend is used to send actions and get observations.
        self.frontend = robot_module.Frontend(self.robot_data)

        #: The logger can be used to log robot data to a file
        self.logger = robot_module.Logger(self.robot_data, 100)

    def initialize(self):
        """Initialize the robot."""
        # Initializes the robot (e.g. performs homing).
        self.backend.initialize()


def demo_print_position(robot):
    """Send zero-torque commands to the robot and print the joint positions.

    :param robot: The robot environment.
    :type robot: Robot
    """
    action = robot.Action()
    print("\nPosition:")
    while True:
        t = robot.frontend.append_desired_action(action)
        pos = robot.frontend.get_observation(t).position
        n_joints = len(pos)
        format_string = "\r" + ", ".join(["{: 6.3f}"] * n_joints)
        print(format_string.format(*pos), end="")
