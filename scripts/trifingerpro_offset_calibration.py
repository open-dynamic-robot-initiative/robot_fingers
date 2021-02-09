#!/usr/bin/env python3
"""Home Offset Calibration for TriFingerPro.

To calibrate the home offset values of a TriFingerPro robot, run this script
and wait until the robot is initialized.  Then move all joints back to the
end-stops to which they moved during initialization.  Make sure each joint is
touching the end-stop bumper but not pushing against it with force.  Then press
enter to determine the home offset.  The home offset values that are printed
can directly be copied to the configuration file of the robot.

"""
import numpy as np

import robot_fingers


# Distance from the zero position (finger pointing straight down) to the
# end-stop.  This is independent of the placement of the encoder disc and thus
# should be the same on all TriFingerPro robots.
_zero_to_endstop = np.array([2.136, 2.442, -2.710] * 3)


# Torque used to find end-stop during homing
# TODO: this could be read from the config file
_homing_torque = [+0.3, +0.3, -0.2] * 3


def main():
    robot = robot_fingers.Robot.create_by_name("trifingerpro_calib")
    robot.initialize()

    # move back to "homing" end-stop
    action = robot.Action(torque=_homing_torque)
    for _ in range(1000):
        t = robot.frontend.append_desired_action(action)
        robot.frontend.wait_until_timeindex(t)

    # release motors, so the joints are not actively pushing against the
    # end-stop anymore
    action = robot.Action()
    for _ in range(1000):
        t = robot.frontend.append_desired_action(action)
        robot.frontend.wait_until_timeindex(t)

    print()
    input("Verify all joints are touching their end-stop. Then press enter.")

    action = robot.Action()
    t = robot.frontend.append_desired_action(action)
    obs = robot.frontend.get_observation(t)

    n_joints = 9
    format_string = ", ".join(["{: 6.3f}"] * n_joints)

    home_to_endstop = obs.position
    endstop_to_zero = -_zero_to_endstop

    home_to_zero = home_to_endstop + endstop_to_zero
    print("End-stop position:", format_string.format(*obs.position))
    print()
    print("Home Offset:", format_string.format(*home_to_zero))


if __name__ == "__main__":
    main()
