#!/usr/bin/env python3
"""Send zero-torque commands to the robot and print joint positions."""
import argparse

import robot_interfaces
import robot_fingers


def main():
    robot_type = {
        "fingerone": (
            robot_interfaces.finger,
            robot_fingers.create_real_finger_backend,
            "finger.yml",
        ),
        "trifingerone": (
            robot_interfaces.trifinger,
            robot_fingers.create_trifinger_backend,
            "trifinger.yml",
        ),
        "fingeredu": (
            robot_interfaces.finger,
            robot_fingers.create_real_finger_backend,
            "fingeredu.yml",
        ),
        "trifingeredu": (
            robot_interfaces.trifinger,
            robot_fingers.create_trifinger_backend,
            "trifingeredu.yml",
        ),
        "trifingerpro": (
            robot_interfaces.trifinger,
            robot_fingers.create_trifinger_backend,
            "trifingerpro.yml",
        ),
        "onejoint": (
            robot_interfaces.one_joint,
            robot_fingers.create_one_joint_backend,
            "onejoint.yml",
        ),
        "twojoint": (
            robot_interfaces.two_joint,
            robot_fingers.create_two_joint_backend,
            "twojoint.yml",
        ),
    }

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("robot_type", choices=robot_type.keys())
    args = parser.parse_args()

    robot = robot_fingers.Robot(*robot_type[args.robot_type])

    robot.initialize()
    robot_fingers.demo_print_position(robot)


if __name__ == "__main__":
    main()
