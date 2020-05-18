#!/usr/bin/env python3
"""Basic demo on how to control position for only some joints.

This demo shows how to run a position controller on only the upper two joints
of the Finger robot while controlling zero-torque on the tip joint.
"""
import os
import numpy as np
import rospkg

import robot_interfaces
import robot_fingers


def main():
    # load the default config file
    config_file_path = os.path.join(
        rospkg.RosPack().get_path("robot_fingers"), "config", "finger.yml"
    )

    # Storage for all observations, actions, etc.
    finger_data = robot_interfaces.finger.SingleProcessData()

    # The backend sends actions from the data to the robot and writes
    # observations from the robot to the data.
    real_finger_backend = robot_fingers.create_real_finger_backend(
        finger_data, config_file_path
    )

    # The frontend is used by the user to get observations and send actions
    finger = robot_interfaces.finger.Frontend(finger_data)

    # Initializes the robot (e.g. performs homing).
    real_finger_backend.initialize()

    while True:
        # set the position for the last joint to NaN to disable the position
        # controller for this joint
        desired_position = [0.7, 1.5, np.nan]

        # set only the position in the action, torque is zero by default
        action = robot_interfaces.finger.Action(position=desired_position)
        t = finger.append_desired_action(action)
        finger.wait_until_time_index(t)


if __name__ == "__main__":
    main()
