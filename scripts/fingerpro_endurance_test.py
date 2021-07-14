#!/usr/bin/env python3
"""Endurance test for single FingerPro.

Moves the finger around to random positions.  The joint ranges are limited a
bit to avoid self-collisions with the base.  Apart from this it is assumed that
the finger can move freely within its valid range without collisions.
"""
import argparse

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

    limit_low = robot.config.soft_position_limits_lower
    limit_up = robot.config.soft_position_limits_upper

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


def execute_trajectory(frontend, positions, timeout=4000, tolerance=0.03):
    """Move joints open loop on a trajectory.

    Args:
        frontend: Robot frontend for sending commands.
        positions: List of joint positions.
        timeout: Maximum number of steps to pursue one step.
        tolerance: Position tolerance within a target position is considered to
            be reached.
    """
    for target in positions:
        action = robot_interfaces.finger.Action(position=target)
        for _ in range(timeout):
            t = frontend.append_desired_action(action)
            obs = frontend.get_observation(t)

            dist = action.position - obs.position

            # stop as soon as the goal is reached
            if np.all(np.linalg.norm(dist) < tolerance):
                break


def push_block_sequence(frontend):
    """Push against the block at the center once from each side.

    The block is expected to be placed exactly below the middle joint, so that
    the finger tip can reach it.
    """

    # first element is trajectory to bring robot into position, second element
    # is push command (which should be held longer)
    sequence = [
        ([(0, 0, 0)], (0.5, 0, 0)),
        (
            [
                (0, 0, 0),
                (0, 0.9, -1.7),
                (0.4, 0.9, -1.7),
                (0.4, 0, 0),
            ],
            (0, 0, 0),
        ),
        (
            [
                (0.5, 0, 0),
                (0.5, 0.9, -1.7),
                (0.5, 0.9, 0),
                (0.27, 0.5, -0.75),
            ],
            (0.27, 0.25, -0.75),
        ),
        (
            [
                (0.27, 0.5, -0.75),
                (0.27, 0.8, -0.75),
                (0.27, 0.8, -2),
                (0.27, -0.5, -2),
                (0.27, -0.5, 0.7),
            ],
            (0.27, -0.25, 0.7),
        ),
    ]

    go_back = [
        (0.27, -0.5, 0.7),
        (0.27, -0.5, -2),
        (0.27, 0.8, -2),
    ]

    for goto_traj, push in sequence:
        execute_trajectory(frontend, goto_traj)

        # wait a moment
        for _ in range(600):
            action = robot_interfaces.finger.Action(position=goto_traj[-1])
            t = frontend.append_desired_action(action)
            frontend.wait_until_timeindex(t)

        # push!
        for _ in range(6000):
            action = robot_interfaces.finger.Action(position=push)
            t = frontend.append_desired_action(action)
            frontend.wait_until_timeindex(t)

    execute_trajectory(frontend, go_back)


def move_on_full_range(push_interval):
    target_positions = [
        [0, 0, 0],
        [0, 1.57, -1.57],
        [0, 2.4, -2.5],
        [2, 1.57, -2.5],
        [-1.57, 1.57, -2.5],
        [0.5, 1.57, -2.5],
        [0.5, -3, 2.5],
        [0, 0, 0],
    ]

    (
        robot_type,
        backend_func,
        config_file,
    ) = robot_fingers.robot.robot_configs["fingerpro"]
    config_file_path = robot_fingers.robot.get_config_dir() / config_file

    n_joints = 3
    config = robot_fingers.FingerConfig.load_config(str(config_file_path))

    # disable position limits
    config.hard_position_limits_lower = [-np.inf] * n_joints
    config.hard_position_limits_upper = [np.inf] * n_joints
    config.soft_position_limits_lower = [-np.inf] * n_joints
    config.soft_position_limits_upper = [np.inf] * n_joints

    robot_data = robot_interfaces.finger.SingleProcessData()
    robot_backend = robot_fingers.create_real_finger_backend(
        robot_data, config
    )
    robot_frontend = robot_interfaces.finger.Frontend(robot_data)

    robot_backend.initialize()

    initialize(robot_frontend)

    time_printer = robot_fingers.utils.TimePrinter()

    i = 0
    while True:
        execute_trajectory(robot_frontend, target_positions)

        if i % push_interval == 0:
            push_block_sequence(robot_frontend)

        # print current date/time every hour, so we can roughly see how long it
        # ran in case it crashes during a long-run-test.
        time_printer.update()

        i += 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--full-range",
        action="store_true",
        help="""Overwrite position limits and move on a hard-coded trajectory
            using the full mechanical range.
        """,
    )
    parser.add_argument(
        "--push-interval",
        type=int,
        metavar="N",
        default=10,
        help="""[Only if --full-range is set]  Perform a push sequence every N
            iterations.  Default is %(default)d.
        """,
    )
    args = parser.parse_args()

    if args.full_range:
        move_on_full_range(push_interval=args.push_interval)
    else:
        move_with_random_positions()


if __name__ == "__main__":
    main()
