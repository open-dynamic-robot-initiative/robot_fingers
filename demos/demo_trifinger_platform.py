#!/usr/bin/env python3
"""Move TriFingerPro to random positions (with collisions)."""
import numpy as np

import robot_interfaces
import robot_fingers


def get_random_position():
    """Generate a random position within a save range."""
    position_min = np.array([-0.33, 0, -2.7] * 3)
    position_max = np.array([1.0, 1.57, 0] * 3)

    position_range = position_max - position_min
    return position_min + np.random.rand(9) * position_range


def main():
    robot = robot_fingers.TriFingerPlatformFrontend()

    while True:
        action = robot_interfaces.trifinger.Action(position=get_random_position())
        for _ in range(1000):
            t = robot.append_desired_action(action)
            robot.wait_until_timeindex(t)

        obs = robot.get_robot_observation(t)
        print(obs.position)


if __name__ == "__main__":
    main()
