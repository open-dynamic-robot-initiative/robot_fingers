#!/usr/bin/env python3
"""Record data while the user is manually moving the robot."""
import argparse
import curses

import robot_fingers

robot = robot_fingers.Robot.create_by_name('trifingerpro')
robot.initialize()

trajectory = []

def record():
    trajectory = []
    t = 0
    while t < 10000:
        t = robot.frontend.append_desired_action(robot.Action())
        robot.frontend.wait_until_timeindex(t)
        trajectory += [robot.frontend.get_observation(t).position]
        
    print(trajectory)


if __name__ == "__main__":
    record()
