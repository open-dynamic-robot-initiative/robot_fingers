#!/usr/bin/env python3
"""Constantly print the readings of the push sensors of TriFingerPro.

Uses the "trifingerpro_calib" setup, so can safely be used for robots where the
home-offsets are not yet configured.
"""
import numpy as np

import robot_fingers


def main():
    robot = robot_fingers.Robot.create_by_name("trifingerpro_calib")
    robot.initialize()

    action = robot.Action()
    print("\nPush Sensors:")
    while True:
        t = robot.frontend.append_desired_action(action)
        pos = robot.frontend.get_observation(t).tip_force
        n_joints = len(pos)
        format_string = "\r" + ", ".join(["{: 6.3f}"] * n_joints)
        print(format_string.format(*pos), end="")


if __name__ == "__main__":
    main()
