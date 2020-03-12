#!/usr/bin/env python3
import signal
import time
import numpy as np
import copy
import sys
from os import path

import progressbar
import rospkg

from robot_interfaces import one_joint
import robot_fingers

from robot_fingers.logger import Logger


N_JOINTS = 1


# Configuration
# ========================================

# Limit of the range in which the joint can move (i.e. should be a little
# bit before hitting the end stop).
POSITION_LIMIT = 2.7

# Gains for the position controller
KP = 5
KD = 0.04


# Number of times the motor hits the endstop in each "hit end stop" phase
NUM_ENDSTOP_HITS = 10

# Number of back and forth movements during each "fixed velocity" phase
NUM_FIXED_VELOCITY_MOVEMENT_STEPS = 150


# Number of times the complete scenario is repeated
#NUM_ITERATIONS = 20
NUM_ITERATIONS = 1


# ========================================





def zero_torque_ctrl(robot, duration, logger=None, print_position=False):
    """Send zero-torque commands for the specified duration."""
    desired_torque = np.zeros(N_JOINTS)
    step = 0

    if logger:
        t = robot.get_current_time_index()
        logger.set_time(t)

    while step < duration:
        step += 1
        t = robot.append_desired_action(desired_torque)
        if print_position:
            print("\rPosition: %10.4f" %
                  robot.get_observation(t).angle[0], end="")

        if logger:
            logger.record(robot)

def go_to(robot, goal_position, steps, hold, logger=None):
    """Go to the goal position with linear profile and hold there.

    :param robot: Robot frontend used to control the robot.
    :param goal_position: Position to which the robot shall move.
    :param steps: Number of steps for the movement.  The velocity of the robot
        depends on the number of steps and the actual distance it has to move.
    :param hold: Number of time steps to hold the motor at the goal position
        once it is reached.
    :param logger: Logger instance to log the data (optional).
    """
    desired_torque = np.zeros(N_JOINTS)

    t = robot.append_desired_action(desired_torque)
    desired_step_position = copy.copy(robot.get_observation(t).angle)

    if logger:
        logger.set_time(t)

    stepsize = (goal_position - desired_step_position) / steps

    for step in range(steps):
        desired_step_position += stepsize
        t = robot.append_desired_action(desired_torque)
        position_error = (desired_step_position -
                          robot.get_observation(t).angle)
        desired_torque = (KP * position_error -
                          KD * robot.get_observation(t).velocity)

        if logger:
            logger.record(robot)

    for step in range(hold):
        t = robot.append_desired_action(desired_torque)
        position_error = goal_position - robot.get_observation(t).angle
        desired_torque = (KP * position_error -
                          KD * robot.get_observation(t).velocity)

        if logger:
            logger.record(robot)

def go_to_zero(robot, steps, hold, logger=None):
    """Go to zero position.  See go_to for description of parameters."""
    go_to(robot, np.zeros(N_JOINTS), steps, hold, logger)

def hit_endstop(robot, desired_torque, hold=0, timeout=5000, logger=None):
    """Hit the end stop with the given torque.

    Applies a constant torque on the joints until velocity drops to near-zero
    (in which case it is assumed that the end stop is reached).

    :param robot: Robot frontend used to control the robot.
    :param desired_torque: Torque that is applied on the joints.
    :param hold: Duration for which the torque is held up after hitting the end
        stop.
    :param timeout: Stop if joint is still moving after this time.
    :param logger: Logger instance to log the data (optional).
    """
    zero_velocity = 0.001
    step = 0
    t = robot.append_desired_action(desired_torque)

    if logger:
        logger.set_time(t)

    while ((np.any(np.abs(robot.get_observation(t).velocity) > zero_velocity) or
           step < 100) and step < timeout):
        t = robot.append_desired_action(desired_torque)

        if logger:
            logger.record(robot)

        step += 1

    for step in range(hold):
        t = robot.append_desired_action(desired_torque)
        robot.get_observation(t)

        if logger:
            logger.record(robot)

def test_if_moves(robot, desired_torque, timeout, logger=None):
    for i in range(timeout):
        t = robot.append_desired_action(desired_torque)
        if logger:
            logger.record(robot)
        # This is a bit hacky: It is assumed that the joints move if they reach
        # a position > 0 within the given time.  Note that this assumes that
        # they start somewhere in the negative range!
        if np.all(robot.get_observation(t).angle > 0):
            return True
    return False


def determine_start_torque(robot, logger):
    """Determine minimum torque to make the joints move.

    Moves the joint to negative position limit and applies a constant torque.
    The motor is considered to be moving if it reaches the positive range
    within a given time frame.  If not, the whole procedure is repeated with a
    increasing torque until the joint moves.
    """
    desired_torque = np.zeros(N_JOINTS)
    t = robot.append_desired_action(desired_torque)
    logger.set_time(t)

    max_torque = 0.4
    stepsize = 0.025
    for trq in np.arange(max_torque, step=stepsize):
        print("test %f Nm" % trq)
        go_to(robot, -POSITION_LIMIT, 1000, 100)
        desired_torque = np.ones(N_JOINTS) * trq
        if test_if_moves(robot, desired_torque, 3000, logger):
            break


def validate_position(robot):
    """Check if measured position is correct.

    Hit the end stop from both sites to check if expected and actual
    position match.
    """
    tolerance = 0.1
    desired_torque = np.ones(N_JOINTS) * 0.15  # 0.3

    angle = [None, None]

    for i, sign in enumerate((+1, -1)):
        hit_endstop(robot, sign * desired_torque)
        t = robot.get_current_time_index()
        angle[i] = robot.get_observation(t).angle

    center = (angle[0] + angle[1]) / 2

    if np.abs(center) > tolerance:
        raise RuntimeError("Unexpected center position."
                           "Expected 0.0, actual is %f" % center)
    else:
        print("Position is okay.")


def hard_direction_change(robot, num_repetitions, torque, logger):
    """Move back and forth by toggling sign of the torque command."""
    # set position limit far enough from the end stop to ensure we don't hit it
    # even when overshooting (we don't want to break the end stop).
    position_limit = 0.6

    direction = +1
    desired_torque = np.ones(N_JOINTS) * torque

    t = robot.append_desired_action(np.zeros(N_JOINTS))
    logger.set_time(t)

    progress = progressbar.ProgressBar()
    for i in progress(range(num_repetitions)):
        step = 0
        while np.all(robot.get_observation(t).angle < position_limit):
            t = robot.append_desired_action(desired_torque)
            logger.record(robot)
            step += 1
            if step > 1000:
                raise RuntimeError("timeout hard_direction_change")

        step = 0
        while np.all(robot.get_observation(t).angle > -position_limit):
            t = robot.append_desired_action(-desired_torque)
            logger.record(robot)
            step += 1
            if step > 1000:
                raise RuntimeError("timeout -hard_direction_change")

    # dampen movement to not hit end stop
    go_to(robot, -position_limit, 10, 100, logger=logger)


def main():

    if len(sys.argv) >= 2:
        log_directory = sys.argv[1]
    else:
        log_directory = "/tmp"

    def log_path(filename):
        return path.join(log_directory, filename)

    # load the default config file
    config_file_path = path.join(
        rospkg.RosPack().get_path("robot_fingers"), "config", "onejoint.yml")

    robot_data = one_joint.SingleProcessData()
    finger_backend = robot_fingers.create_one_joint_backend(robot_data,
                                                          config_file_path)
    robot = one_joint.Frontend(robot_data)
    logger = Logger()


    # dump logger data in case the script is killed with SIGINT or SIGQUIT
    def signal_handler(signum, stack):
        if logger.data:
            logger.name += "_aborted"
            logger.dump()
        sys.exit(1)
        #raise RuntimeError("Killed by signal %d" % signum)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGQUIT, signal_handler)


    # rotate without end stop
    #logger.start_new_recording(log_path("move_fixed_velocity_rotate"))
    #goal_position = 60
    ## move to goal position within 2000 ms and wait there for 100 ms
    #go_to(robot, goal_position, 20000, 100, logger)
    #logger.dump()

    #return


    finger_backend.initialize()
    print("initialization finished")
    go_to_zero(robot, 1000, 2000)

    #zero_torque_ctrl(robot, 99999999, print_position=True)
    #return

    print("initial position validation")
    validate_position(robot)

    go_to_zero(robot, 1000, 2000)


    #[(0, 0.0),
    #  (1, 0.18),
    #  (2, 0.36),
    #  (3, 0.54),
    #  (4, 0.72),
    #  (5, 0.8999999999999999),
    #  (6, 1.08),
    #  (7, 1.26),
    #  (8, 1.44),
    #  (9, 1.6199999999999999),
    #  (10, 1.7999999999999998)]


    # Careful, dangerous!
    #logger.start_new_recording(log_path("hit_endstop"))
    #hit_torque = np.ones(N_JOINTS) * 1.26
    #print("Start to push")
    #hit_endstop(robot, hit_torque, hold=100, logger=logger)
    #hit_endstop(robot, -hit_torque, hold=100, logger=logger)
    #logger.dump()


    for iteration in range (NUM_ITERATIONS):
        print("START TEST ITERATION %d" % iteration)

        #print("Determine torque to start movement.")
        #logger.start_new_recording(log_path("start_torque_%d" % iteration))
        #determine_start_torque(robot, logger)
        #logger.dump()

        print("Switch directions with high torque")
        #low_trq = 0.36
        low_trq = 0.2
        #currents = range(5, 19)
        currents = range(5, 15)
        for current in currents:
            trq = current * (0.02 * 9)
            print("A = %d (trq = %f)" % (current, trq))
            go_to(robot, -POSITION_LIMIT, 500, 10, logger)
            logger.start_new_recording(log_path("hard_switch_directions_%dA_%d"
                                                % (current, iteration)))
            hard_direction_change(robot, 2, trq, logger)

            t = robot.get_current_time_index()
            if np.any(np.abs(robot.get_observation(t).angle) >
                      POSITION_LIMIT):
                print("ERROR: Position limit exceeded!")
                return

            hard_direction_change(robot, 10, low_trq, logger)
            logger.dump()

            # Determine torque to start movement
            #logger.start_new_recording(log_path("start_torque_after_switch_direction_%dA_%d"
            #                                    % (current, iteration)))
            #determine_start_torque(robot, logger)
            #logger.dump()


        print("position validation after switch directions")
        validate_position(robot)


        # skip the following tests
        continue





        print("Hit the end stop...")

        trq = 1.8
        logger.start_new_recording(log_path("hit_endstop_%.3f_%d" % (trq, iteration)))
        hit_torque = np.ones(N_JOINTS) * trq
        progress = progressbar.ProgressBar()
        for i in progress(range(NUM_ENDSTOP_HITS)):
            hit_torque *= -1
            hit_endstop(robot, hit_torque, hold=10, logger=logger)
            #hit_endstop(robot, hit_torque, logger=logger)
            #zero_torque_ctrl(robot, 10, logger)
        logger.dump()

        #logger.start_new_recording(log_path("hit_endstop_0.2_%d" % iteration))
        #hit_torque = np.ones(N_JOINTS) * 0.2
        #for i in range(NUM_ENDSTOP_HITS):
        #    hit_torque *= -1
        #    hit_endstop(robot, hit_torque, logger=logger)
        #    zero_torque_ctrl(robot, 10, logger)
        #logger.dump()

        #logger.start_new_recording(log_path("hit_endstop_0.4_%d" % iteration))
        #hit_torque = np.ones(N_JOINTS) * 0.4
        #for i in range(NUM_ENDSTOP_HITS):
        #    hit_torque *= -1
        #    hit_endstop(robot, hit_torque, logger=logger)
        #    zero_torque_ctrl(robot, 10, logger)
        #logger.dump()

        print("position validation after hitting")
        validate_position(robot)


        print("Move with fixed velocity...")

        goal_position = POSITION_LIMIT

        logger.start_new_recording(log_path("move_fixed_velocity_2000_%d" % iteration))
        progress = progressbar.ProgressBar()
        for i in progress(range(NUM_FIXED_VELOCITY_MOVEMENT_STEPS)):
            goal_position *= -1
            # move to goal position within 2000 ms and wait there for 100 ms
            go_to(robot, goal_position, 2000, 100, logger)
        logger.dump()

        #print("validate position")
        #validate_position(robot)

        #logger.start_new_recording(log_path("move_fixed_velocity_1000_%d" % iteration))
        #for i in range(NUM_FIXED_VELOCITY_MOVEMENT_STEPS):
        #    goal_position *= -1
        #    go_to(robot, goal_position, 1000, 100, logger)
        #logger.dump()

        #print("validate position")
        #validate_position(robot)

        #logger.start_new_recording(log_path("move_fixed_velocity_500_%d" % iteration))
        #for i in range(NUM_FIXED_VELOCITY_MOVEMENT_STEPS):
        #    goal_position *= -1
        #    go_to(robot, goal_position, 500, 100, logger)
        #logger.dump()


        print("final position validation")
        validate_position(robot)

        go_to_zero(robot, 1000, 3000)


if __name__ == "__main__":
    main()
