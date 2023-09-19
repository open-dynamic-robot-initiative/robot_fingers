#!/usr/bin/env python3
"""Demo showing how to use the robot data logger."""
import numpy as np

from robot_interfaces import finger
import robot_fingers


def main():
    finger_data = finger.SingleProcessData()
    finger_backend = robot_fingers.create_fake_finger_backend(finger_data)
    finger_frontend = finger.Frontend(finger_data)

    desired_torque = np.zeros(3)

    # The buffer limit should cover at least the number of steps done in
    # the experiment.
    buffer_limit = 11000
    filename = "log.csv"

    # initialise and start the logger
    logger = finger.Logger(finger_data, buffer_limit)
    logger.start()

    # run the robot for a while
    for _ in range(10000):
        t = finger_frontend.append_desired_action(finger.Action(torque=desired_torque))
        finger_frontend.wait_until_timeindex(t)

    # stop logger and write data to file
    logger.stop_and_save(filename, finger.Logger.Format.CSV)

    finger_backend.request_shutdown()


if __name__ == "__main__":
    main()
