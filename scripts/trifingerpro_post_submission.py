#!/usr/bin/env python3
"""Perform "post submission" actions on TriFingerPro.

Performs the following actins:

- run some self-tests to check if the robot is behaving as expected
- move the fingers on a previously recorded trajectory to randomize the
  position of the cube
- use the cameras to check if the cube is still inside the arena (not
  implemented yet)
"""
import argparse
import os
import sys
import typing

import numpy as np
import pandas
from ament_index_python.packages import get_package_share_directory
import tomli

import robot_interfaces
import robot_fingers
import trifinger_object_tracking.py_tricamera_types as tricamera
import trifinger_object_tracking.py_object_tracker


# Distance from the zero position (finger pointing straight down) to the
# end-stop.
_zero_to_endstop = np.array([2.136, 2.442, -2.710] * 3)

# Maximum torque with signs same as for end-stop search
_max_torque_against_homing_endstop = [+0.4, +0.4, -0.4] * 3

# Tolerance when checking if expected position is reached
_position_tolerance = 0.1

_submission_system_config_file = "/etc/trifingerpro/submission_system.toml"


def load_object_type() -> typing.Optional[str]:
    with open(_submission_system_config_file, "rb") as f:
        config = tomli.load(f)

    try:
        return config["post_submission"]["object_type"]
    except KeyError:
        print("ERROR: failed to load object from config file.")
        return None


def get_robot_config_without_position_limits() -> (
    robot_fingers.TriFingerConfig
):
    """Get TriFingerPro configuration without position limits.

    Loads the TriFingerPro configuration from the default config file and
    disables the position limits.
    """
    # get the path to the TriFingerPro config file
    _, _, config_file = robot_fingers.robot.robot_configs["trifingerpro"]
    config_file_path = robot_fingers.robot.get_config_dir() / config_file

    # load the config
    config = robot_fingers.TriFingerConfig.load_config(str(config_file_path))

    # disable position limits
    n_joints = 9
    config.hard_position_limits_lower = [-np.inf] * n_joints
    config.hard_position_limits_upper = [np.inf] * n_joints
    config.soft_position_limits_lower = [-np.inf] * n_joints
    config.soft_position_limits_upper = [np.inf] * n_joints

    return config


def end_stop_check(robot: robot_fingers.Robot) -> None:
    """Move robot to endstop, using constant torque and verify its position.

    Applies a constant torque for a fixed time to move to the end-stop.  If
    everything is okay, the robot should be at the end-stop position after this
    time.  If it is not, trigger a fault.

    Then move back to the initial position in a controlled way and again verify
    that the goal was reached successfully.

    Args:
        robot:  Initialized robot instance of the TriFingerPro.
    """
    # go to the "homing" end-stop (using the same torque as during homing)
    action = robot.Action(
        torque=robot.config.calibration.endstop_search_torques_Nm
    )
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


def run_self_test(robot: robot_fingers.Robot) -> None:
    position_tolerance = 0.2
    push_sensor_threshold = 0.5

    initial_pose = [0.0, 1.1, -1.9] * 3

    reachable_goals = [
        [0.9, 1.5, -2.6] * 3,
        [-0.3, 1.5, -1.7] * 3,
    ]

    unreachable_goals = [
        [0.0, 0.0, 0.0] * 3,
        [-0.5, 1.5, 0.0] * 3,
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


def reset_object(robot, trajectory_file):
    """Replay a recorded trajectory to reset/randomise the object pose.

    Args:
        robot: The robot object.
        trajectory_file: Path to a CSV file defining the trajectory.
    """
    trajectory_file = os.path.join(
        get_package_share_directory("robot_fingers"),
        "config",
        trajectory_file,
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


def check_if_cube_is_there(object_type: str):
    """Verify that the cube is still inside the arena."""

    object_models = {
        "cube": "cube_v2",
        "cuboid": "cuboid_2x2x8_v2",
    }

    camera_data = tricamera.SingleProcessData(history_size=5)
    model = trifinger_object_tracking.py_object_tracker.get_model_by_name(
        object_models[object_type]
    )
    camera_driver = tricamera.TriCameraObjectTrackerDriver(
        "camera60", "camera180", "camera300", model
    )
    camera_backend = tricamera.Backend(camera_driver, camera_data)
    camera_frontend = tricamera.Frontend(camera_data)

    observation = camera_frontend.get_latest_observation()
    if observation.object_pose.confidence == 0:
        print("Cube not found.")
        sys.exit(2)
    else:
        print("Cube found.")

    camera_backend.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--object",
        type=str,
        metavar="OBJECT_TYPE",
        choices=["cube", "cuboid", "dice", "auto"],
        help="""Specify with which object the robot is equipped (if any).  If
            set to "auto", the object type is read from the submission system
            configuration.
        """,
    )
    parser.add_argument(
        "--reset",
        action="store_true",
        help="""Execute a trajectory to reset the object.  Only valid if
            --object is set.
        """,
    )
    args = parser.parse_args()

    if args.object == "auto":
        args.object = load_object_type()

    config = get_robot_config_without_position_limits()
    robot = robot_fingers.Robot(
        robot_interfaces.trifinger,
        robot_fingers.create_trifinger_backend,
        config,
    )
    robot.initialize()

    print("End stop test")
    end_stop_check(robot)
    print("Position reachability test")
    run_self_test(robot)

    if args.reset:
        if args.object == "cube":
            print("Reset cube position")
            reset_object(
                robot, "trifingerpro_shuffle_cube_trajectory_fast.csv"
            )
        elif args.object == "cuboid":
            print("Reset cuboid position")
            reset_object(robot, "trifingerpro_recenter_cuboid_2x2x8.csv")
        elif args.object == "dice":
            print("Shuffle dice positions")
            reset_object(robot, "trifingerpro_shuffle_dice_trajectory.csv")

    # terminate the robot
    del robot

    if args.object in ["cube", "cuboid"]:
        print("Check if cube/cuboid is found")
        check_if_cube_is_there(args.object)


if __name__ == "__main__":
    main()
