#!/usr/bin/env python3
"""Record data while the user is manually moving the robot."""
import argparse
import curses

# %%


# %%

import robot_fingers

robot = robot_fingers.Robot.create_by_name('trifingerpro')
robot.initialize()

trajectory = []

def record():
    trajectory[:] = []
    t = 0
    for _ in range(10**4):
        t = robot.frontend.append_desired_action(robot.Action())
        robot.frontend.wait_until_timeindex(t)
        trajectory += [robot.frontend.get_observation(t).position]
        

def play():
    for position in trajectory:    
        action = robot.Action(position=position)
        robot.append_desired_action(action)


if __name__ == "__main__":
    
    while True:
        print('enter key')
        key = input()
        if key == 'r':
            print('recording')
            record()
            print(trajectory)
        elif key == 'p':
            print('playing')
            play()
        else:
            print('invalid key')
    


