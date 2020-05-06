#!/usr/bin/env python3
"""Endurance test for single FingerEdu.

Moves the finger around to random positions.  The joint ranges are limited to
ensure that the finger does not hit, e.g. the electronics above it.
"""
import time
import os
import numpy as np
import rospkg

import robot_interfaces
import robot_fingers


def get_random_position():
    """Generate a random position within a save range."""
    position_min = np.array([-0.6, -1.5, -2])
    position_max = np.array([1, 1.5, 2])

    position_range = position_max - position_min

    return position_min + np.random.rand(3) * position_range


def demo_position_commands(finger):
    """Demo for directly sending position commands."""
    last_time_print = 0

    while True:
        # Run a position controller that randomly changes the desired position
        # every 300 steps.
        # One time step corresponds to roughly 1 ms.

        desired_position = get_random_position()
        for _ in range(300):
            # Appends a torque command ("action") to the action queue.
            # Returns the time step at which the action is going to be
            # executed.
            action = robot_interfaces.finger.Action(position=desired_position)
            t = finger.append_desired_action(action)
            finger.wait_until_time_index(t)

        # print current date/time every hour, so we can roughly see how long it
        # ran in case it crashes during a long-run-test.
        now = time.time()
        if now - last_time_print > 3600:
            print(time.strftime("%F %T"))
            last_time_print = now


def main():
    input(
        "Manually move the finger such that it points straight down."
        "  Then press Enter to initialize."
    )

    config_file_path = os.path.join(
        rospkg.RosPack().get_path("robot_fingers"), "config", "fingeredu.yml"
    )

    finger_data = robot_interfaces.finger.SingleProcessData()
    backend = robot_fingers.create_real_finger_backend(
        finger_data, config_file_path
    )
    finger = robot_interfaces.finger.Frontend(finger_data)
    backend.initialize()

    input(
        "Verify that finger is pointing straight down."
        "  Press Enter to continue."
    )

    demo_position_commands(finger)


if __name__ == "__main__":
    main()
