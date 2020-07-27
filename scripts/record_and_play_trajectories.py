#!/usr/bin/env python3
"""Record data while the user is manually moving the robot."""
import argparse
import curses
import numpy as np
import robot_fingers
import time


def stay_at_current_position():
    t = robot.frontend.get_current_timeindex()
    position = robot.frontend.get_observation(t).position
    action = robot.Action(position=position)
    robot.frontend.append_desired_action(action)

def release():
    t = robot.frontend.append_desired_action(robot.Action())



def record(trajectory):
    trajectory[:] = []
    t = 0
    while True:
        t = robot.frontend.append_desired_action(robot.Action())
        robot.frontend.wait_until_timeindex(t)
        trajectory += [robot.frontend.get_observation(t).position]

        if len(trajectory) > 3000:
            if np.abs(trajectory[-1] - trajectory[-2000]).sum() < 0.05:
                trajectory[-2000:] = [] 
                return

    stay_at_current_position()


def play(trajectory, factor):
    for position in trajectory[::factor]:
        action = robot.Action(position=position)
        t = robot.frontend.append_desired_action(action)
        robot.frontend.wait_until_timeindex(t - 10)


    # robot.frontend.append_desired_action(robot.Action())


if __name__ == "__main__":
    robot = robot_fingers.Robot.create_by_name('trifingeredu')
    robot.initialize()
    robot.frontend.append_desired_action(robot.Action())

    stay_at_current_position()
    print('staying done')

    trajectory = []
    while True:
        print('enter key')
        key = input()
        if key == 'r':
            time.sleep(2) # sleep to sec to give user some time       
            print('recording')
            record(trajectory)
            print('returning to  initial position')
            play(trajectory, -1)
        elif key in list(map(str, range(1, 100))):
            print('playing with factor ' + key)
            play(trajectory, int(key))
            print('returning to  initial position')
            play(trajectory, -1)
        elif key == 'n':
            trajectory[:] = []
            release()
        else:
            print('invalid key')
