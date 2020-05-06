#!/usr/bin/env python3
"""Calibration of home offset for various finger robots."""
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
    }

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("robot_type", choices=robot_type.keys())
    args = parser.parse_args()

    robot = robot_fingers.Robot(*robot_type[args.robot_type])

    print("")
    print("")
    input("Manually move robot to zero position.  Then press Enter.")
    robot.initialize()

    print("")
    print("Finished. The 'Offset' corresponds to the home offset.")


if __name__ == "__main__":
    main()
