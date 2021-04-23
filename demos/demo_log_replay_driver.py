#!/usr/bin/env python3
"""FIXME
"""
import argparse

import numpy as np

import robot_interfaces
import robot_fingers.log_replay_driver


def get_random_position():
    """Generate a random position within a save range."""
    position_min = np.array([-0.33, 0.0, -2.7] * 3)
    position_max = np.array([1.0, 1.57, 0.0] * 3)

    return np.random.uniform(position_min, position_max)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("robot_log_file")
    parser.add_argument("camera_log_file")
    parser.add_argument("--output-robot-log")
    args = parser.parse_args()

    # read log to get number of steps
    log = robot_interfaces.trifinger.BinaryLogReader(args.robot_log_file)
    num_steps = len(log.data)
    del log
    print("Log length:", num_steps)

    driver = robot_fingers.log_replay_driver.TriFingerPlatformLogReplayDriver(
        args.robot_log_file, args.camera_log_file
    )
    finger_data = robot_interfaces.trifinger.SingleProcessData(
        history_size=num_steps + 10
    )
    backend = robot_fingers.log_replay_driver.create_backend(
        driver, finger_data, max_number_of_actions=num_steps
    )
    frontend = robot_interfaces.trifinger.Frontend(finger_data)
    logger = robot_interfaces.trifinger.Logger(finger_data, buffer_limit=1)

    backend.initialize()

    Action = robot_interfaces.trifinger.Action
    try:
        for i in range(num_steps):
            desired_position = get_random_position()
            t = frontend.append_desired_action(Action(position=desired_position))
            current_position = frontend.get_observation(t).position

            # print current position from time to time
            if i % 1000 == 0:
                print("Position: %s" % current_position)
    except Exception as e:
        print("Error:", e)

    if args.output_robot_log:
        logger.save_current_robot_data(args.output_robot_log)


if __name__ == "__main__":
    main()
