#!/usr/bin/env python3
"""Send zero-torque commands to the robot and create a log"""

import time
import numpy as np

from robot_interfaces import finger
import blmc_robots

def main():

    finger_data = finger.Data()
    finger_backend = blmc_robots.create_fake_finger_backend(finger_data)
    finger_frontend = finger.Frontend(finger_data)

    desired_torque = np.zeros(3)

    block_size = 100
    filename = "log.csv"

    finger_logger = finger.Logger(finger_data, block_size)
    finger_logger.start(filename)

    while True:
        for _ in range(1000):
            t = finger_frontend.append_desired_action(
                finger.Action(torque=desired_torque))

            pos = finger_frontend.get_observation(t).position

if __name__ == "__main__":
    main()
