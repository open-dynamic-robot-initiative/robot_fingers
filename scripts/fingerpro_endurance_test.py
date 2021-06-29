#!/usr/bin/env python3
"""Endurance test for single FingerPro.

Moves the finger around to random positions.  The joint ranges are limited a
bit to avoid self-collisions with the base.  Apart from this it is assumed that
the finger can move freely within its valid range without collisions.
"""
import argparse
import os

import numpy as np

import robot_interfaces
import robot_fingers


def get_random_position():
    """Generate a random position within a save range."""
    position_min = np.array([-0.6, -1.5, -2])
    position_max = np.array([1, 0.6, 2])

    position_range = position_max - position_min

    return position_min + np.random.rand(3) * position_range


def initialize(frontend):
    # first move to zero position, so it is easy to detect if there is some
    # initialisation issue
    action = robot_interfaces.finger.Action(position=[0, 0, 0])
    for _ in range(300):
        t = frontend.append_desired_action(action)
        frontend.wait_until_timeindex(t)
    input(
        "Verify that finger is pointing straight down."
        "  Press Enter to continue or Ctrl+C to abort."
    )


def move_with_random_positions():
    robot = robot_fingers.Robot.create_by_name("fingerpro")
    robot.initialize()

    # first move to zero position, so it is easy to detect if there is some
    # initialisation issue
    initialize(robot.frontend)

    limit_low = robot.config["soft_position_limits_lower"]
    limit_up = robot.config["soft_position_limits_upper"]

    time_printer = robot_fingers.utils.TimePrinter()

    while True:
        # get random position within the robot's valid range
        position = np.random.uniform(limit_low, limit_up)

        # reject configurations that might collide with the robot's base
        if position[1] > 1.2 and position[2] > -1.1:
            continue

        for _ in range(300):
            action = robot_interfaces.finger.Action(position=position)
            t = robot.frontend.append_desired_action(action)
            robot.frontend.wait_until_timeindex(t)

        # print current date/time every hour, so we can roughly see how long it
        # ran in case it crashes during a long-run-test.
        time_printer.update()


def move_on_full_range():
    target_positions = [
        [0, 0, 0],
        [0, 1.57, -1.57],
        [0, 2.2, -2.5],
        [2, 1.57, -2.5],
        [-1.57, 1.57, -2.5],
        [0.5, 1.57, -2.5],
        [0.5, -3, 2.5],
    ]
    tolerance = 0.03

    (robot_type, backend_func, config_file) = robot_fingers.robot.robot_configs["fingerpro"]
    config_file_path = robot_fingers.robot.get_config_dir() / config_file

    n_joints = 3
    config = robot_fingers.FingerConfig.load_config(str(config_file_path))

    # disable position limits
    config.hard_position_limits_lower = [-np.inf] * n_joints
    config.hard_position_limits_upper = [np.inf] * n_joints
    config.soft_position_limits_lower = [-np.inf] * n_joints
    config.soft_position_limits_upper = [np.inf] * n_joints

    robot_data = robot_interfaces.finger.SingleProcessData()
    robot_backend = robot_fingers.create_real_finger_backend(robot_data, config)
    robot_frontend = robot_interfaces.finger.Frontend(robot_data)

    robot_backend.initialize()

    initialize(robot_frontend)

    time_printer = robot_fingers.utils.TimePrinter()

    idx = 0
    while True:
        action = robot_interfaces.finger.Action(position=target_positions[idx])
        for _ in range(4000):
            t = robot_frontend.append_desired_action(action)
            obs = robot_frontend.get_observation(t)

            dist = action.position - obs.position

            # stop as soon as the goal is reached
            if np.all(np.linalg.norm(dist) < tolerance):
                break

        # print current date/time every hour, so we can roughly see how long it
        # ran in case it crashes during a long-run-test.
        time_printer.update()

        idx = (idx + 1) % len(target_positions)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--full-range",
        action="store_true",
        help="""Overwrite position limits and move on a hard-coded trajectory
            using the full mechanical range.
        """,
    )
    args = parser.parse_args()

    if args.full_range:
        move_on_full_range()
    else:
        move_with_random_positions()

if __name__ == "__main__":
    main()
