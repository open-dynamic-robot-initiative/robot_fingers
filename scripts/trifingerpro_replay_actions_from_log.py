#!/usr/bin/env python3
"""
Load a TriFingerPro log file and apply the actions from the log on the robot.
"""
import argparse
import pathlib
import sys

import robot_interfaces
import robot_fingers


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "log_file",
        type=pathlib.Path,
        help="Path to the binary robot log file.",
    )
    args = parser.parse_args()

    log = robot_interfaces.trifinger.BinaryLogReader(str(args.log_file))

    robot = robot_fingers.Robot.create_by_name("trifingerpro")
    robot.initialize()

    for log_entry in log.data:
        t = robot.frontend.append_desired_action(log_entry.desired_action)

        # print torque of the action
        des_action = log_entry.desired_action
        app_action = log_entry.applied_action
        vector_fmt = ", ".join(["{: .4f}"] * 9)
        print(chr(27) + "[2J")
        print(f"[{t: 4d}] Desired: [{vector_fmt.format(*des_action.torque)}]")
        print(f"[{t: 4d}] Applied: [{vector_fmt.format(*app_action.torque)}]")

        robot.frontend.wait_until_timeindex(t)

    return 0


if __name__ == "__main__":
    sys.exit(main())
