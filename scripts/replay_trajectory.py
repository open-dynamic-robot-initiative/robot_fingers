#!/usr/bin/env python3
"""Move the robot on the trajectory of a previously recorded log file."""
import argparse

import pandas

import robot_fingers


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "robot_type",
        choices=robot_fingers.Robot.get_supported_robots(),
        help="Name of the robot.",
    )
    parser.add_argument("logfile", type=str, help="Path to the log file.")
    parser.add_argument(
        "--speed", "-s", type=float, default=1.0, help="Playback speed."
    )
    args = parser.parse_args()

    data = pandas.read_csv(
        args.logfile, delim_whitespace=True, header=0, low_memory=False
    )

    # determine number of joints
    data_keys = []
    key_pattern = "observation_position_{}"
    i = 0
    while key_pattern.format(i) in data:
        data_keys.append(key_pattern.format(i))
        i += 1

    # extract the positions from the recorded data
    positions = data[data_keys].to_numpy()
    num_positions = len(positions)
    print("Replay {} positions".format(num_positions))

    # initialize robot
    robot = robot_fingers.Robot.create_by_name(args.robot_type)
    robot.initialize()

    # move robot to the recorded positions.  At speed = 1, every position is
    # executed once (= original speed), at speed < 1, positions are repeated
    # (slower playback), at speed > 1, some positions are skipped (faster
    # playback).
    step = 0
    while int(step) < num_positions:
        position = positions[int(step)]
        step += args.speed

        action = robot.Action(position=position)
        t = robot.frontend.append_desired_action(action)
        robot.frontend.wait_until_timeindex(t)


if __name__ == "__main__":
    main()
