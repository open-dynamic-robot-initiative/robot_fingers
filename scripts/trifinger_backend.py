#!/usr/bin/env python3
"""Run TriFinger back-end using multi-process robot data."""
import argparse
import os

import rospkg

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
        "--cameras",
        "-c",
        action="store_true",
        help="Run camera backend.",
    )
    parser.add_argument(
        "--fake-object-tracker",
        action="store_true",
        help="Run fake object tracker backend",
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

    if args.cameras:
        print("Start camera backend")
        import trifinger_cameras

        CAMERA_TIME_SERIES_LENGTH = 100

        camera_data = trifinger_cameras.tricamera.MultiProcessData(
            "tricamera", True, CAMERA_TIME_SERIES_LENGTH
        )
        camera_driver = trifinger_cameras.tricamera.TriCameraDriver(
            "camera60", "camera180", "camera300"
        )
        camera_backend = trifinger_cameras.tricamera.Backend(  # noqa
            camera_driver, camera_data
        )

        print("Camera backend ready.")

    # Use robot-dependent config file
    config_file_path = "/etc/trifingerpro/trifingerpro.yml"

    # Storage for all observations, actions, etc.
    robot_data = robot_interfaces.trifinger.MultiProcessData("trifinger", True)

    if args.robot_logfile:
        robot_logger = robot_interfaces.trifinger.Logger(robot_data)

    # The backend sends actions from the data to the robot and writes
    # observations from the robot to the data.
    backend = robot_fingers.create_trifinger_backend(
        robot_data,
        config_file_path,
        max_number_of_actions=args.max_number_of_actions,
    )

    # Initializes the robot (e.g. performs homing).
    backend.initialize()

    if args.cameras and args.camera_logfile:
        camera_fps = 100
        robot_rate_hz = 1000
        episode_length_s = args.max_number_of_actions / robot_rate_hz
        # Compute camera log size based on number of robot actions plus a
        # 10% buffer
        log_size = int(camera_fps * episode_length_s * 1.1)

        print("Start camera logger with buffer size", log_size)
        camera_logger = trifinger_cameras.tricamera.Logger(camera_data, log_size)
        camera_logger.start()

    if args.fake_object_tracker:
        import trifinger_object_tracking.py_object_tracker as object_tracker
        object_tracker_data = object_tracker.Data("object_tracker", True)
        object_tracker_backend = object_tracker.FakeBackend(  # noqa
            object_tracker_data
        )

    backend.wait_until_terminated()

    if args.cameras and args.camera_logfile:
        print("Save recorded camera data to file {}".format(args.camera_logfile))
        camera_logger.stop_and_save(args.camera_logfile)

    if args.robot_logfile:
        print("Save robot data to file {}".format(args.robot_logfile))
        if args.max_number_of_actions:
            end_index = args.max_number_of_actions
        else:
            end_index = -1
        robot_logger.write_current_buffer(
            args.robot_logfile, start_index=0, end_index=end_index
        )


if __name__ == "__main__":
    main()
