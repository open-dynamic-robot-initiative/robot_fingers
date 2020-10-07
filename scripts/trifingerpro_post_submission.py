#!/usr/bin/env python3
"""Perform "post submission" actions on TriFingerPro.

Performs the following actins:

- run some self-tests to check if the robot is behaving as expected
- move the fingers on a previously recorded trajectory to randomize the
  position of the cube
- use the cameras to check if the cube is still inside the arena (not
  implemented yet)
"""
import os
import sys

import numpy as np
import pandas
import rospkg

import robot_fingers
import trifinger_cameras


def run_self_test(robot):
    position_tolerance = 0.2
    push_sensor_threshold = 0.5

    initial_pose = [0, 1.1, -1.9] * 3

    reachable_goals = [
        [0.75, 1.2, -2.3] * 3,
        [0.9, 1.5, -2.6] * 3,
        initial_pose,
        [-0.3, 1.5, -1.7] * 3,
    ]

    unreachable_goals = [
        [0, 0, 0] * 3,
        [-0.5, 1.5, 0] * 3,
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
            # sys.exit(1)

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
            # sys.exit(1)

    print("Test successful.")


def shuffle_cube(robot):
    """Replay a recorded trajectory to shuffle the position of the cube."""
    trajectory_file = os.path.join(
        rospkg.RosPack().get_path("robot_fingers"),
        "config",
        "trifingerpro_shuffle_cube_trajectory.csv",
    )
    data = pandas.read_csv(
        trajectory_file, delim_whitespace=True, header=0, low_memory=False
    )

    # determine number of joints
    key_pattern = "observation_position_{}"
    data_keys = [key_pattern.format(i) for i in range(9)]

    # extract the positions from the recorded data
    positions = data[data_keys].to_numpy()

    for position in positions:
        action = robot.Action(position=position)
        t = robot.frontend.append_desired_action(action)
        robot.frontend.wait_until_timeindex(t)


def check_if_cube_is_there():
    """Verify that the cube is still inside the arena."""
    camera_data = trifinger_cameras.tricamera.SingleProcessData()
    camera_driver = trifinger_cameras.tricamera.TriCameraDriver(
        "camera60", "camera180", "camera300"
    )
    camera_backend = trifinger_cameras.tricamera.Backend(  # noqa
        camera_driver, camera_data
    )
    camera_frontend = trifinger_cameras.tricamera.Frontend(camera_data)
    observation = camera_frontend.get_latest_observation()  # noqa

    # TODO check if cube is found in camera image (maybe just see if object
    # tracking founds something?)


def main():
    robot = robot_fingers.Robot.create_by_name("trifingerpro")
    robot.initialize()

    run_self_test(robot)
    shuffle_cube(robot)
    check_if_cube_is_there()


if __name__ == "__main__":
    main()
