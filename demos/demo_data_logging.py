#!/usr/bin/env python3
"""Send zero-torque commands to the robot and create a log"""

import numpy as np

from robot_interfaces import finger
import robot_fingers


def main():

    finger_data = finger.SingleProcessData()
    finger_backend = robot_fingers.create_fake_finger_backend(finger_data)
    finger_frontend = finger.Frontend(finger_data)

    desired_torque = np.zeros(3)

    block_size = 100
    filename = "log.csv"

    finger_logger = finger.Logger(finger_data, block_size)
    finger_logger.start(filename)

    while True:
        for _ in range(1000):
            t = finger_frontend.append_desired_action(
                finger.Action(torque=desired_torque)
            )

            finger_frontend.wait_for_timeindex(t)

    finger_backend.shutdown()


if __name__ == "__main__":
    main()
