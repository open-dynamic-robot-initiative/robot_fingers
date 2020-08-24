#!/usr/bin/env python3
"""Move TriFingerPro to random positions (with collisions)."""
import numpy as np
import time

import robot_fingers


def get_random_position():
    """Generate a random position within a save range."""
    #position_min = np.array([-0.9, -1.57, -2.7] * 3)
    #position_max = np.array([1.4, 1.57, 0] * 3)
    position_min = np.array([-0.33, 0, -2.7] * 3)
    position_max = np.array([1.0, 1.57, 0] * 3)

    position_range = position_max - position_min
    return position_min + np.random.rand(9) * position_range


def main():
    robot = robot_fingers.Robot.create_by_name("trifingerpro")
    robot.initialize()

    last_time_print = 0

    while True:
        action = robot.Action(position=get_random_position())
        for _ in range(1000):
            t = robot.frontend.append_desired_action(action)
            pos = robot.frontend.get_observation(t).position

        # print current date/time every hour, so we can roughly see how long it
        # ran in case it crashes during a long-run-test.
        now = time.time()
        if (now - last_time_print > 3600):
            print(time.strftime("%F %T"))
            last_time_print = now


if __name__ == "__main__":
    main()
