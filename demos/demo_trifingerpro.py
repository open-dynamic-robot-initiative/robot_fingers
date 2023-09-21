#!/usr/bin/env python3
"""Demo script for the TriFingerPro robot

Moves the TriFingerPro robot with a hard-coded choreography for show-casing and
testing.
"""
import argparse

import robot_interfaces
import robot_fingers
from robot_fingers.utils import min_jerk_trajectory


def run_choreography(frontend):
    """Move the legs in some hard-coded choreography."""

    def perform_step(start, goal):
        # one step should take 1 second, so repeat action 1000 times
        STEPS = 1000

        # use min_jerk_trajectory to generate smooth trajectories (we could also just
        # use `Action(position=goal)` and append that for several steps until the goal
        # is reached but it would result in harsher movements).
        for step_position in min_jerk_trajectory(start, goal, STEPS):
            t = frontend.append_desired_action(
                robot_interfaces.trifinger.Action(position=step_position)
            )
            frontend.wait_until_timeindex(t)

        # return final position
        observation = frontend.get_observation(t)
        return observation.position

    # Desired joint positions for the steps.  Specify for a single finger and multiply
    # by 3, so we get a list of length 9 with identical joint positions for all three
    # fingers.
    pose_initial = [0, 0.9, -1.7] * 3
    pose_intermediate = [0.75, 1.2, -2.3] * 3
    pose_up = [1.3, 1.5, -2.6] * 3
    pose_other_side_up = [-0.8, 1.5, -1.7] * 3

    time_printer = robot_fingers.utils.TimePrinter()

    # send a zero-torque action to start the backend loop, so we can get the initial
    # position
    t = frontend.append_desired_action(robot_interfaces.trifinger.Action())
    observation = frontend.get_observation(t)
    current_position = observation.position

    while True:
        # initial pose
        current_position = perform_step(current_position, pose_initial)
        current_position = perform_step(current_position, pose_intermediate)
        current_position = perform_step(current_position, pose_up)
        current_position = perform_step(current_position, pose_initial)
        current_position = perform_step(current_position, pose_other_side_up)

        # print current date/time every hour, so we can roughly see how long it
        # ran in case it crashes during a long-run-test.
        time_printer.update()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--multi-process",
        action="store_true",
        help="""If set run only frontend with multi-process
                        robot data.  Otherwise run everything within a single
                        process.""",
    )
    parser.add_argument("--log", type=str)
    args = parser.parse_args()

    if args.multi_process:
        # In multi-process case assume that the backend is running in a
        # separate process and only set up the frontend here.
        robot_data = robot_interfaces.trifinger.MultiProcessData("trifinger", False)
        frontend = robot_interfaces.trifinger.Frontend(robot_data)
    else:
        # In single-process case run both frontend and backend in this process
        # (using the `Robot` helper class).
        robot = robot_fingers.Robot.create_by_name("trifingerpro")
        if args.log:
            logger = robot_interfaces.trifinger.Logger(robot.robot_data, 100)
            logger.start(args.log)

        robot.initialize()
        frontend = robot.frontend

    # move around
    run_choreography(frontend)


if __name__ == "__main__":
    main()
