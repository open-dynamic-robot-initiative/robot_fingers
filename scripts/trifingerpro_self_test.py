#!/usr/bin/env python3
"""Run some self-tests to check if the robot is behaving as expected."""
import sys
import numpy as np

import robot_fingers


def main():
    robot = robot_fingers.Robot.create_by_name("trifingerpro")
    robot.initialize()

    position_tolerance = 0.2
    push_sensor_threshold = 0.2

    initial_pose = [0, 0.9, -1.7] * 3

    reachable_goals = [
        [0.75, 1.2, -2.3] * 3,
        [1.3, 1.5, -2.6] * 3,
        initial_pose,
        [-0.8, 1.5, -1.7] * 3,
    ]

    unreachable_goals = [
        [0, 0, 0] * 3,
        [0, 1.5, 0] * 3,
    ]

    for goal in reachable_goals:
        action = robot.Action(position=goal)
        for _ in range(1000):
            t = robot.frontend.append_desired_action(action)
            robot.frontend.wait_until_timeindex(t)

        observation = robot.frontend.get_observation(t)

        # verify that goal is reached
        if np.linalg.norm(goal - observation.position) > position_tolerance:
            print("Robot did not reach goal position")
            print("Desired position: {}".format(goal))
            print("Actual position: {}".format(observation.position))
            sys.exit(1)

        if (observation.tip_force > push_sensor_threshold).any():
            print("Push sensor reports high value in non-contact situation.")
            print("Sensor value: {}".format(observation.tip_force))
            print("Desired position: {}".format(goal))
            print("Actual position: {}".format(observation.position))
            sys.exit(1)

    for goal in unreachable_goals:
        # move to initial position first
        action = robot.Action(position=initial_pose)
        for _ in range(1000):
            t = robot.frontend.append_desired_action(action)
            robot.frontend.wait_until_timeindex(t)

        action = robot.Action(position=goal)
        for _ in range(1000):
            t = robot.frontend.append_desired_action(action)
            robot.frontend.wait_until_timeindex(t)

        observation = robot.frontend.get_observation(t)

        # verify that goal is reached
        if np.linalg.norm(goal - observation.position) < position_tolerance:
            print("Robot reached a goal which should not be reachable.")
            print("Desired position: {}".format(goal))
            print("Actual position: {}".format(observation.position))
            sys.exit(1)

        if (observation.tip_force < push_sensor_threshold).any():
            print("Push sensor reports low value in contact situation.")
            print("Sensor value: {}".format(observation.tip_force))
            print("Desired position: {}".format(goal))
            print("Actual position: {}".format(observation.position))
            sys.exit(1)

    print("Test successful.")


if __name__ == "__main__":
    main()
