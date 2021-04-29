#!/usr/bin/env python3
"""Run TriFinger Log Replay back-end using follower multi-process robot data.

This requires a leader robot data to be running in a separate process!
"""
import argparse
import math
import sys

# ROS imports
import rclpy

import robot_interfaces
import robot_fingers.log_replay_driver
import trifinger_object_tracking.py_tricamera_types as tricamera
from robot_fingers.ros import NotificationNode


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("robot_log_file", type=str, help="The robot log file.")
    parser.add_argument(
        "camera_log_file", type=str, help="The camera log file."
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
    args = parser.parse_args()

    rclpy.init()
    node = NotificationNode("trifinger_backend")

    logger = node.get_logger()

    # read log to get number of steps
    log = robot_interfaces.trifinger.BinaryLogReader(args.robot_log_file)
    num_steps = len(log.data)
    del log
    logger.info("Log length: %d" % num_steps)

    driver = robot_fingers.log_replay_driver.TriFingerPlatformLogReplayDriver(
        args.robot_log_file, args.camera_log_file
    )

    # robot pipeline
    robot_data = robot_interfaces.trifinger.MultiProcessData(
        "trifinger", False
    )
    robot_backend = robot_fingers.log_replay_driver.create_backend(
        driver,
        robot_data,
        first_action_timeout=args.first_action_timeout,
        max_number_of_actions=num_steps,
    )

    # camera pipeline
    camera_data = tricamera.MultiProcessData("tricamera", False)
    camera_backend = tricamera.Backend(driver, camera_data)
    camera_backend  # to silence unused warning

    robot_backend.initialize()

    logger.info("Log Replay backend is ready")

    # send ready signal
    node.publish_status("READY")

    # wait until backend terminates or shutdown request is received
    while robot_backend.is_running():
        rclpy.spin_once(node, timeout_sec=1)
        if node.shutdown_requested:
            robot_backend.request_shutdown()
            robot_backend.wait_until_terminated()
            break

    termination_reason = robot_backend.get_termination_reason()
    logger.debug("Backend termination reason: %d" % termination_reason)

    rclpy.shutdown()
    if termination_reason < 0:
        # negate code as exit codes should be positive
        return -termination_reason
    else:
        return 0


if __name__ == "__main__":
    sys.exit(main())
