#!/usr/bin/env python3
"""Demo script for the Solo8 robot

Moves the Solo8 robot with a hard-coded choreography for show-casing and
testing.
"""
import argparse
import numpy as np

import robot_interfaces
import robot_fingers


def run_choreography(frontend):
    """Move the legs in some hard-coded choreography."""

    def perform_step(position):
        # one step should take 1 second, so repeat action 1000 times
        for _ in range(1):
            t = frontend.append_desired_action(
                robot_interfaces.solo_eight.Action(position=position)
            )
            frontend.wait_until_timeindex(t)

    time_printer = robot_fingers.utils.TimePrinter()

    t = 0
    offset = np.random.uniform(-0.1, 0.1, size=8)
    while True:
        action = np.sin(t * 0.01 + offset) * 0.5
        perform_step(action)

        time_printer.update()

        t += 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--multi-process",
        action="store_true",
        help="""If set run only frontend with multi-process
                        robot data.  Otherwise run everything within a single
                        process.""",
    )
    args = parser.parse_args()

    if args.multi_process:
        # In multi-process case assume that the backend is running in a
        # separate process and only set up the frontend here.
        robot_data = robot_interfaces.solo_eight.MultiProcessData("solo8", False)
        frontend = robot_interfaces.solo_eight.Frontend(robot_data)
    else:
        # In single-process case run both frontend and backend in this process
        # (using the `Robot` helper class).
        robot = robot_fingers.Robot(
            robot_interfaces.solo_eight,
            robot_fingers.create_solo_eight_backend,
            "soloeight.yml",
        )
        robot.initialize()
        frontend = robot.frontend

    # move around
    print("Running")
    run_choreography(frontend)


if __name__ == "__main__":
    main()
