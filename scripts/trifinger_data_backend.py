#!/usr/bin/env python3
"""Run TriFinger Data Backend

Runs the leader multi-processing robot/sensor data and loggers.
"""

import argparse
import sys

# ROS imports
import rclpy

import robot_interfaces
from robot_fingers.ros import NotificationNode
from trifinger_cameras import camera


ROBOT_TIME_SERIES_LENGTH = 1000
ROBOT_RATE_HZ = 1000


def parse_args() -> argparse.Namespace:
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

    if args.camera_logfile and not (args.cameras or args.cameras_with_tracker):
        parser.error("--camera-logfile requires --cameras or --cameras-with-tracker")

    return args


def main() -> int:
    args = parse_args()

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

        camera_settings = camera.Settings()
        camera_rate_hz = camera_settings.get_tricamera_driver_settings().frame_rate_fps
        logger.debug("Loaded camera frame rate from settings: %f fps" % camera_rate_hz)

        # make sure camera time series covers at least the same duration as the robot
        # time series (add some margin to avoid problems)
        time_series_length_seconds = ROBOT_TIME_SERIES_LENGTH / ROBOT_RATE_HZ
        length_margin_ratio = 1.5
        camera_time_series_length = int(
            camera_rate_hz * time_series_length_seconds * length_margin_ratio
        )
        logger.debug("Set camera time series length to %d" % camera_time_series_length)

        camera_data = tricamera.MultiProcessData(
            "tricamera", True, camera_time_series_length
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
        # Compute camera log size based on number of robot actions plus some buffer
        log_size = int(camera_rate_hz * episode_length_s * buffer_length_factor)

        logger.info("Initialize camera logger with buffer size %d" % log_size)
        camera_logger = tricamera.Logger(camera_data, log_size)

    logger.info("Data backend is ready")

    # send ready signal
    node.publish_status("READY")

    if cameras_enabled and args.camera_logfile:
        # wait for first action to be sent by the user (but make sure to not
        # block when shutdown is requested)
        while (
            not robot_data.desired_action.wait_for_timeindex(0, max_duration_s=1)
            and not node.shutdown_requested
        ):
            rclpy.spin_once(node, timeout_sec=0)

        camera_logger.start()
        logger.info("Start camera logging")

    while not node.shutdown_requested:
        rclpy.spin_once(node)

    logger.debug("Received shutdown signal")

    if cameras_enabled and args.camera_logfile:
        logger.info("Save recorded camera data to file %s" % args.camera_logfile)
        camera_logger.stop_and_save(args.camera_logfile)

    if args.robot_logfile:
        logger.info("Save robot data to file %s" % args.robot_logfile)
        robot_logger.stop_and_save(
            args.robot_logfile, robot_interfaces.trifinger.Logger.Format.BINARY
        )

    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
