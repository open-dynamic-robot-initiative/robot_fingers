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
import yaml

import robot_interfaces
import robot_fingers
import trifinger_object_tracking.py_tricamera_types as tricamera


# Distance from the zero position (finger pointing straight down) to the
# end-stop.  This is independent of the placement of the encoder disc and
# thus should be the same on all TriFingerPro robots.
# TODO: this should actually be the same for all three fingers!  Needs to
#       be fixed in the calibration script, though.
_zero_to_endstop = np.array(
    [2.112, 2.399, -2.714, 2.118, 2.471, -2.694, 2.179, 2.456, -2.723]
)

# Torque used to find end-stop during homing
# TODO: this could be read from the config file
_homing_torque = [+0.3, +0.3, -0.2] * 3

# Maximum torque with signs same as for end-stop search
_max_torque_against_homing_endstop = [+0.4, +0.4, -0.4] * 3

# Tolerance when checking if expected position is reached
_position_tolerance = 0.1


def get_robot_config(
    position_limits=True,
    robot_config_file="/etc/trifingerpro/trifingerpro.yml",
) -> str:
    """Get path to robot configuration file.

    This may be the file specified by robot_config_file or a temporary copy
    with modifications, based on other arguments.

    Args:
        position_limits:  If True, the original configuration is used with the
            position limits as they are defined there.  If false, a temporary
            configuration file is created where the position limits are
            removed.
        robot_config_file:  Path to the original robot configuration file.
    """
    if position_limits:
        return robot_config_file
    else:
        with open(robot_config_file, "r") as fh:
            config = yaml.load(fh)

        # remove position limits
        config["hard_position_limits_lower"] = [-np.inf] * 9
        config["hard_position_limits_upper"] = [+np.inf] * 9
        del config["soft_position_limits_lower"]
        del config["soft_position_limits_upper"]

        tmp_config_file = "/tmp/trifingerpro.yml"
        with open(tmp_config_file, "w") as fh:
            yaml.dump(config, fh)

        return tmp_config_file


def end_stop_check(robot: robot_fingers.Robot):
    """Move robot to endstop, using constant torque and verify its position.

    Applies a constant torque for a fixed time to move to the end-stop.  If
    everything is okay, the robot should be at the end-stop position after this
    time.  If it is not, trigger a fault.

    Then move back to the initial position in a controlled way and again verify
    that the goal was reached successfully.

    Args:
        robot:  Initialized robot instance of the TriFingerPro.
    """
    # go to the "homing" end-stop
    action = robot.Action(torque=_homing_torque)
    for _ in range(2000):
        t = robot.frontend.append_desired_action(action)
        robot.frontend.wait_until_timeindex(t)

    # push with maximum torque for a moment for slipping joint detection
    action = robot.Action(torque=_max_torque_against_homing_endstop)
    for _ in range(1000):
        t = robot.frontend.append_desired_action(action)
        robot.frontend.wait_until_timeindex(t)

    # release motors, so the joints are not actively pushing against the
    # end-stop anymore
    action = robot.Action()
    for _ in range(500):
        t = robot.frontend.append_desired_action(action)
        robot.frontend.wait_until_timeindex(t)

    observation = robot.frontend.get_observation(t)

    if (
        np.linalg.norm(_zero_to_endstop - observation.position)
        > _position_tolerance
    ):
        print("End stop not at expected position.")
        print("Expected position: {}".format(_zero_to_endstop))
        print("Actual position: {}".format(observation.position))
        sys.exit(1)

    # move back joint-by-joint
    goals = [
        (1000, np.array([0, np.nan, np.nan] * 3)),
        (500, np.array([0, 1.1, np.nan] * 3)),
        (500, np.array([0, 1.1, -1.9] * 3)),
    ]
    for duration, goal in goals:
        action = robot.Action(position=goal)
        for _ in range(duration):
            t = robot.frontend.append_desired_action(action)
            robot.frontend.wait_until_timeindex(t)

    observation = robot.frontend.get_observation(t)

    # verify that goal is reached
    if np.linalg.norm(goal - observation.position) > _position_tolerance:
        print("Robot did not reach goal position")
        print("Desired position: {}".format(goal))
        print("Actual position: {}".format(observation.position))
        sys.exit(1)


def run_self_test(robot):
    position_tolerance = 0.2
    push_sensor_threshold = 0.5

    initial_pose = [0, 1.1, -1.9] * 3

    reachable_goals = [
        [0.9, 1.5, -2.6] * 3,
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
        "trifingerpro_shuffle_cube_trajectory_fast.csv",
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
    camera_data = tricamera.SingleProcessData(history_size=5)
    camera_driver = tricamera.TriCameraObjectTrackerDriver(
        "camera60", "camera180", "camera300"
    )
    camera_backend = tricamera.Backend(camera_driver, camera_data)  # noqa
    camera_frontend = tricamera.Frontend(camera_data)

    observation = camera_frontend.get_latest_observation()
    if observation.object_pose.confidence == 0:
        print("Cube not found.")
        sys.exit(2)
    else:
        print("Cube found.")


def main():
    config_file = get_robot_config(position_limits=False)

    # robot = robot_fingers.Robot.create_by_name("trifingerpro")
    robot = robot_fingers.Robot(
        robot_interfaces.trifinger,
        robot_fingers.create_trifinger_backend,
        config_file,
    )
    robot.initialize()

    print("End stop test")
    end_stop_check(robot)
    print("Position reachability test")
    run_self_test(robot)
    print("Reset cube position")
    shuffle_cube(robot)

    # terminate the robot
    del robot

    print("Check if cube is found")
    check_if_cube_is_there()


if __name__ == "__main__":
    main()
