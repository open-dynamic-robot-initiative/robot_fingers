#!/usr/bin/env python3
"""Run Solo8 back-end using multi-process robot data."""
import os
import argparse
import logging
import math
import pathlib
import sys
from ament_index_python.packages import get_package_share_directory

import robot_interfaces
import robot_fingers


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--max-number-of-actions",
        "-a",
        type=int,
        required=True,
        help="""Maximum numbers of actions that are processed.  After this the
            backend shuts down automatically.
        """,
    )
    parser.add_argument(
        "--first-action-timeout",
        "-t",
        type=float,
        default=math.inf,
        help="""Timeout (in seconds) for reception of first action after
            starting the backend.  If not set, the timeout is disabled.
        """,
    )
    parser.add_argument(
        "--robot-logfile",
        type=str,
        help="""Path to a file to which the robot data log is written.  If not
            specified, no log is generated.
        """,
    )
    parser.add_argument(
        "--ready-indicator",
        type=str,
        metavar="READY_INDICATOR_FILE",
        help="""Path to a file that will be created once the backend is ready
            and will be deleted again when it stops (before storing the logs).
        """,
    )
    args = parser.parse_args()

    log_handler = logging.StreamHandler(sys.stdout)
    logging.basicConfig(
        format="[Solo8_BACKEND %(levelname)s %(asctime)s] %(message)s",
        level=logging.DEBUG,
        handlers=[log_handler],
    )

    logging.info("Start robot backend")

    # Use robot-dependent config file
    config_file_path = os.path.join(
        get_package_share_directory("robot_fingers"), "config", "soloeight.yml"
    )

    # Storage for all observations, actions, etc.
    history_size = args.max_number_of_actions + 1
    robot_data = robot_interfaces.solo_eight.MultiProcessData(
        "solo8", True, history_size=history_size
    )

    if args.robot_logfile:
        robot_logger = robot_interfaces.solo_eight.Logger(robot_data)

    # The backend sends actions from the data to the robot and writes
    # observations from the robot to the data.
    backend = robot_fingers.create_solo_eight_backend(
        robot_data,
        config_file_path,
        first_action_timeout=args.first_action_timeout,
        max_number_of_actions=args.max_number_of_actions,
    )

    # Initializes the robot (e.g. performs homing).
    backend.initialize()

    logging.info("Robot backend is ready")

    # if specified, create the "ready indicator" file to indicate that the
    # backend is ready
    if args.ready_indicator:
        pathlib.Path(args.ready_indicator).touch()

    termination_reason = backend.wait_until_terminated()
    logging.debug("Backend termination reason: %d", termination_reason)

    # delete the ready indicator file to indicate that the backend has shut
    # down
    if args.ready_indicator:
        pathlib.Path(args.ready_indicator).unlink()

    if args.robot_logfile:
        logging.info("Save robot data to file %s", args.robot_logfile)
        if args.max_number_of_actions:
            end_index = args.max_number_of_actions
        else:
            end_index = -1

        robot_logger.write_current_buffer_binary(
            args.robot_logfile, start_index=0, end_index=end_index
        )

    if termination_reason < 0:
        # negate code as exit codes should be positive
        return -termination_reason
    else:
        return 0


if __name__ == "__main__":
    returncode = main()
    sys.exit(returncode)
