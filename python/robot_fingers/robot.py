"""Classes and functions to easily set up robot demo scripts."""
import os
import rospkg


class Robot:

    def __init__(self, robot_module, create_backend_function, config_file_name):
        """Initialize the robot environment (backend and frontend).

        :param robot_module: The module that defines the robot classes (i.e.
            `Data`, `Frontend`, `Backend`, ...)
        :param create_backend_function: Function to create the robot backend.
        :param config_file_name: Name of the config file (expected to be
            located in robot_fingers/config).
        """
        # convenience mapping of the Action type
        self.Action = robot_module.Action

        # Use the default config file from the robot_fingers package
        config_file_path = os.path.join(
            rospkg.RosPack().get_path("robot_fingers"), "config",
            config_file_name)

        # Storage for all observations, actions, etc.
        self.robot_data = robot_module.SingleProcessData()

        # The backend sends actions from the data to the robot and writes
        # observations from the robot to the data.
        self.backend = create_backend_function(self.robot_data,
                                               config_file_path)

        # The frontend is used by the user to get observations and send actions
        self.frontend = robot_module.Frontend(self.robot_data)

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
    while True:
        t = robot.frontend.append_desired_action(action)
        pos = robot.frontend.get_observation(t).position
        n_joints = len(pos)
        format_string = "\r" + ", ".join(["{: 6.3f}"] * n_joints)
        print(format_string.format(*pos), end="")
