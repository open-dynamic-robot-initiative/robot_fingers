#!/usr/bin/env python3
"""Run TriFinger Data Backend.

Runs the leader multi-processing robot/sensor data and loggers.
"""
import argparse
import sys

# ROS imports
import rclpy

import robot_interfaces
from robot_fingers.ros import NotificationNode


# make sure camera time series covers at least one second
CAMERA_TIME_SERIES_LENGTH = 15

ROBOT_TIME_SERIES_LENGTH = 1000

# Frame rate of the cameras (needed to determine camera logger buffer size)
CAMERA_FPS = 10
# Update rate of the robot (needed to determine camera logger buffer size)
ROBOT_RATE_HZ = 1000


def main() -> int:
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
    camera_group = parser.add_mutually_exclusive_group()
    camera_group.add_argument(
        "--cameras",
        "-c",
        action="store_true",
        help="Run camera backend.",
    )
    camera_group.add_argument(
        "--cameras-with-tracker",
        action="store_true",
        help="Run camera backend with integrated object tracker.",
    )
    parser.add_argument(
        "--robot-logfile",
        type=str,
        help="""Path to a file to which the robot data log is written.  If not
            specified, no log is generated.
        """,
    )
    parser.add_argument(
        "--camera-logfile",
        type=str,
        help="""Path to a file to which the camera data is written.  If not
            specified, no log is generated.
        """,
    )
    args = parser.parse_args()

    rclpy.init()
    node = NotificationNode("trifinger_data")

    logger = node.get_logger()

    cameras_enabled = False
    if args.cameras:
        cameras_enabled = True
        from trifinger_cameras import tricamera
    elif args.cameras_with_tracker:
        cameras_enabled = True
        import trifinger_object_tracking.py_tricamera_types as tricamera

    if cameras_enabled:
        logger.info("Start camera data")

        camera_data = tricamera.MultiProcessData(
            "tricamera", True, CAMERA_TIME_SERIES_LENGTH
        )

    logger.info("Start robot data")

    # Storage for all observations, actions, etc.
    robot_data = robot_interfaces.trifinger.MultiProcessData(
        "trifinger", True, history_size=ROBOT_TIME_SERIES_LENGTH
    )

    if args.robot_logfile:
        robot_logger = robot_interfaces.trifinger.Logger(
            robot_data, buffer_limit=args.max_number_of_actions
        )
        robot_logger.start()

    if cameras_enabled and args.camera_logfile:
        # make the logger buffer a bit bigger as needed to be on the safe side
        buffer_length_factor = 1.5

        episode_length_s = args.max_number_of_actions / ROBOT_RATE_HZ
        # Compute camera log size based on number of robot actions plus some
        # buffer
        log_size = int(CAMERA_FPS * episode_length_s * buffer_length_factor)

        logger.info("Initialize camera logger with buffer size %d", log_size)
        camera_logger = tricamera.Logger(camera_data, log_size)

    logger.info("Data backend is ready")

    # send ready signal
    node.publish_status("READY")

    if cameras_enabled and args.camera_logfile:
        # wait for first action to be sent by the user (but make sure to not
        # block when shutdown is requested)
        while (
            not robot_data.desired_action.wait_for_timeindex(
                0, max_duration_s=1
            )
            and not node.shutdown_requested
        ):
            rclpy.spin_once(node, timeout_sec=0)

        camera_logger.start()
        logger.info("Start camera logging")

    while not node.shutdown_requested:
        rclpy.spin_once(node)

    logger.debug("Received shutdown signal")

    if cameras_enabled and args.camera_logfile:
        logger.info(
            "Save recorded camera data to file %s", args.camera_logfile
        )
        camera_logger.stop_and_save(args.camera_logfile)

    if args.robot_logfile:
        logger.info("Save robot data to file %s", args.robot_logfile)
        robot_logger.stop_and_save(
            args.robot_logfile, robot_interfaces.trifinger.Logger.Format.BINARY
        )

    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
