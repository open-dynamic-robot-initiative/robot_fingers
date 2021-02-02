#!/usr/bin/env python3
"""Basic demo on how to use pyBullet through the robot_interfaces pipeline.

This demo illustrates how to use the pyBullet simulation in the backend of the
robot_interfaces pipeline.  When used like this, the same code can be executed
in simulation and on the real robot with only changing a single line, namely
the one for creating the backend.
"""
import argparse
import numpy as np

import robot_interfaces
import robot_fingers.pybullet_drivers
from trifinger_simulation import finger_types_data


def get_random_position(num_fingers=1):
    """Generate a random position within a save range."""
    position_min = np.array([-1, -1, -2] * num_fingers)
    position_max = np.array([1, 1, 2] * num_fingers)

    position_range = position_max - position_min

    return position_min + np.random.rand(3 * num_fingers) * position_range


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--finger-type",
        required=True,
        choices=finger_types_data.get_valid_finger_types(),
        help="Specify valid finger type.",
    )
    parser.add_argument(
        "--real-time-mode",
        "-r",
        action="store_true",
        help="""Run simulation in real time.  If not set,
                        the simulation runs as fast as possible.
                        """,
    )
    args = parser.parse_args()

    # select the correct types/functions based on which robot is used
    num_fingers = finger_types_data.get_number_of_fingers(args.finger_type)
    if num_fingers == 1:
        finger_types = robot_interfaces.finger
        create_backend = (
            robot_fingers.pybullet_drivers.create_single_finger_backend
        )
    elif num_fingers == 3:
        finger_types = robot_interfaces.trifinger
        create_backend = (
            robot_fingers.pybullet_drivers.create_trifinger_backend
        )

    robot_data = finger_types.SingleProcessData()

    # Create backend with the simulation as driver.
    # Simply replace this line by creating a backend for the real robot to run
    # the same code on the real robot.
    backend = create_backend(
        robot_data, real_time_mode=args.real_time_mode, visualize=True
    )

    frontend = finger_types.Frontend(robot_data)

    backend.initialize()

    # Simple example application that moves the finger to random positions.
    while True:
        action = finger_types.Action(position=get_random_position(num_fingers))
        for _ in range(300):
            t = frontend.append_desired_action(action)
            frontend.wait_until_timeindex(t)

        # print current position from time to time
        current_position = frontend.get_observation(t).position
        print("Position: %s" % current_position)


if __name__ == "__main__":
    main()
