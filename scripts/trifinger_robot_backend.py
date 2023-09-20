#!/usr/bin/env python3
"""Run TriFinger Robot back-end using follower multi-process robot data.

This requires a leader robot data to be running in a separate process!
"""
import argparse
import functools
import math
import sys

# ROS imports
import rclpy

import robot_interfaces
import robot_fingers

from robot_fingers.ros import NotificationNode


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
        "--fail-on-incomplete-run",
        action="store_true",
        help="""Exit with non-zero return code if the backend is terminated
            before reaching `--max-number-of-actions`.
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
        "--object",
        type=str,
        default="cube_v2",
        help="""Name of the object model used for object tracking.
            Only used if --cameras-with-tracker is set.  Default: %(default)s.
        """,
    )
    args = parser.parse_args()

    rclpy.init()
    node = NotificationNode("trifinger_backend")

    logger = node.get_logger()

    cameras_enabled = False
    if args.cameras:
        cameras_enabled = True
        from trifinger_cameras import tricamera

        CameraDriver = tricamera.TriCameraDriver
    elif args.cameras_with_tracker:
        cameras_enabled = True
        import trifinger_object_tracking.py_tricamera_types as tricamera
        import trifinger_object_tracking.py_object_tracker

        model = trifinger_object_tracking.py_object_tracker.get_model_by_name(
            args.object
        )
        CameraDriver = functools.partial(
            tricamera.TriCameraObjectTrackerDriver, cube_model=model
        )

    if cameras_enabled:
        logger.info("Start camera backend")

        camera_data = tricamera.MultiProcessData("tricamera", False)
        camera_driver = CameraDriver("camera60", "camera180", "camera300")
        camera_backend = tricamera.Backend(camera_driver, camera_data)

        logger.info("Camera backend ready.")

    logger.info("Start robot backend")

    # Use robot-dependent config file
    config_file_path = "/etc/trifingerpro/trifingerpro.yml"

    # Storage for all observations, actions, etc.
    robot_data = robot_interfaces.trifinger.MultiProcessData("trifinger", False)

    # The backend sends actions from the data to the robot and writes
    # observations from the robot to the data.
    backend = robot_fingers.create_trifinger_backend(
        robot_data,
        config_file_path,
        first_action_timeout=args.first_action_timeout,
        max_number_of_actions=args.max_number_of_actions,
    )

    # Initializes the robot (e.g. performs homing).
    backend.initialize()

    logger.info("Robot backend is ready")

    # send ready signal
    node.publish_status("READY")

    # wait until backend terminates or shutdown request is received
    while backend.is_running():
        rclpy.spin_once(node, timeout_sec=1)
        if node.shutdown_requested:
            backend.request_shutdown()
            backend.wait_until_terminated()
            break

    if cameras_enabled:
        camera_backend.shutdown()

    termination_reason = backend.get_termination_reason()
    logger.debug("Backend termination reason: %d" % termination_reason)

    rclpy.shutdown()

    TermReason = robot_interfaces.RobotBackendTerminationReason
    if (
        args.fail_on_incomplete_run
        and termination_reason != TermReason.MAXIMUM_NUMBER_OF_ACTIONS_REACHED
    ):
        # if --fail-on-incomplete-run is set any reason other than having
        # reached the action limit is considered a failure.
        logger.fatal(
            "Expected termination reason %d (%s) but got %d"
            % (
                TermReason.MAXIMUM_NUMBER_OF_ACTIONS_REACHED,
                TermReason.MAXIMUM_NUMBER_OF_ACTIONS_REACHED.name,
                termination_reason,
            )
        )

        return 20
    elif termination_reason < 0:
        # negative termination reason means there was an error

        # negate code as exit codes should be positive
        return -termination_reason
    else:
        return 0


if __name__ == "__main__":
    sys.exit(main())
