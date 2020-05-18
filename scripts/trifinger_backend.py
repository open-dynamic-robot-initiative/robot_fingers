#!/usr/bin/env python3
"""Run TriFinger back-end using multi-process robot data."""
import os

import rospkg

import robot_interfaces
import robot_fingers


def main():
    # Use the default config file from the robot_fingers package
    config_file_path = os.path.join(
        rospkg.RosPack().get_path("robot_fingers"), "config", "trifinger.yml"
    )

    # Storage for all observations, actions, etc.
    robot_data = robot_interfaces.trifinger.MultiProcessData("trifinger", True)

    # The backend sends actions from the data to the robot and writes
    # observations from the robot to the data.
    backend = robot_fingers.create_trifinger_backend(
        robot_data, config_file_path
    )

    # Initializes the robot (e.g. performs homing).
    backend.initialize()

    # FIXME add function to check if back-end is still running
    while True:
        pass


if __name__ == "__main__":
    main()
