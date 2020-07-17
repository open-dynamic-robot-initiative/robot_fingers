#!/usr/bin/env python3
"""Send zero-torque commands to the robot and print joint positions."""
import argparse

import robot_fingers


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "robot_type", choices=robot_fingers.Robot.get_supported_robots()
    )
    args = parser.parse_args()

    robot = robot_fingers.Robot.create_by_name(args.robot_type)

    robot.initialize()
    robot_fingers.demo_print_position(robot)


if __name__ == "__main__":
    main()
