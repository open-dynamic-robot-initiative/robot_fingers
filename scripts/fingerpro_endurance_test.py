#!/usr/bin/env python3
"""Endurance test for single FingerPro.

Moves the finger around to random positions.  The joint ranges are limited a
bit to avoid self-collisions with the base.  Apart from this it is assumed that
the finger can move freely within its valid range without collisions.
"""
import numpy as np

import robot_interfaces
import robot_fingers


def get_random_position():
    """Generate a random position within a save range."""
    position_min = np.array([-0.6, -1.5, -2])
    position_max = np.array([1, 0.6, 2])

    position_range = position_max - position_min

    return position_min + np.random.rand(3) * position_range


def main():
    robot = robot_fingers.Robot.create_by_name("fingerpro")
    robot.initialize()

    # first move to zero position, so it is easy to detect if there is some
    # initialisation issue
    action = robot_interfaces.finger.Action(position=[0, 0, 0])
    for _ in range(300):
        t = robot.frontend.append_desired_action(action)
        robot.frontend.wait_until_timeindex(t)
    input(
        "Verify that finger is pointing straight down."
        "  Press Enter to continue or Ctrl+C to abort."
    )

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


if __name__ == "__main__":
    main()
