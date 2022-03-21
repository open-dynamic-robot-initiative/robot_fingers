#!/usr/bin/env python3
"""Run TriFinger back-end using multi-process robot data."""
import argparse
import logging
import math
import pathlib
import sys
import typing

import robot_interfaces
import robot_fingers


def find_robot_config_file(
    config_dir: pathlib.Path,
    filenames: typing.Sequence[str] = ("trifinger.yml", "trifingerpro.yml"),
) -> pathlib.Path:
    """Find robot config file using a list of allowed filenames.

    Checks if any of the files listed in ``filenames`` exists in ``config_dir``
    and returns the first match.  If none of the files exists, a FileNotFound
    error is thrown.
    """
    for file in (config_dir / f for f in filenames):
        if file.exists():
            return file

    raise FileNotFoundError(
        "None of the files %s/{%s} exists" % (config_dir, ",".join(filenames))
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--max-number-of-actions",
        "-a",
        type=int,
        default=0,
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
    parser.add_argument(
        "--ready-indicator",
        type=str,
        metavar="READY_INDICATOR_FILE",
        help="""Path to a file that will be created once the backend is ready
            and will be deleted again when it stops (before storing the logs).
        """,
    )
    parser.add_argument(
        "--config-dir",
        type=pathlib.Path,
        default="/etc/trifingerpro",
        help="""Path to the directory in which robot and camera configuration
            are found.  Default: %(default)s
        """,
    )
    args = parser.parse_args()

    log_handler = logging.StreamHandler(sys.stdout)
    logging.basicConfig(
        format="[TRIFINGER_BACKEND %(levelname)s %(asctime)s] %(message)s",
        level=logging.DEBUG,
        handlers=[log_handler],
    )

    if not args.config_dir.exists():
        logging.fatal("Config directory %s does not exist")
        return 1

    cameras_enabled = False
    if args.cameras:
        cameras_enabled = True
        from trifinger_cameras import tricamera

        CameraDriver = tricamera.TriCameraDriver
    elif args.cameras_with_tracker:
        cameras_enabled = True
        import trifinger_object_tracking.py_tricamera_types as tricamera

        CameraDriver = tricamera.TriCameraObjectTrackerDriver

    if cameras_enabled:
        logging.info("Start camera backend")

        # make sure camera time series covers at least one second
        CAMERA_TIME_SERIES_LENGTH = 15

        camera_data = tricamera.MultiProcessData(
            "tricamera", True, CAMERA_TIME_SERIES_LENGTH
        )
        camera_driver = CameraDriver("camera60", "camera180", "camera300")
        camera_backend = tricamera.Backend(camera_driver, camera_data)

        logging.info("Camera backend ready.")

    logging.info("Start robot backend")

    # Use robot-dependent config file
    config_file_path = find_robot_config_file(args.config_dir)

    # Storage for all observations, actions, etc.
    history_size = args.max_number_of_actions + 1
    robot_data = robot_interfaces.trifinger.MultiProcessData(
        "trifinger", True, history_size=history_size
    )

    if args.robot_logfile:
        robot_logger = robot_interfaces.trifinger.Logger(robot_data)

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

    logging.info("Robot backend is ready")

    if cameras_enabled and args.camera_logfile:
        camera_fps = 10
        robot_rate_hz = 1000
        # make the logger buffer a bit bigger as needed to be on the safe side
        buffer_length_factor = 1.5

        episode_length_s = args.max_number_of_actions / robot_rate_hz
        # Compute camera log size based on number of robot actions plus a
        # 10% buffer
        log_size = int(camera_fps * episode_length_s * buffer_length_factor)

        logging.info("Initialize camera logger with buffer size %d", log_size)
        camera_logger = tricamera.Logger(camera_data, log_size)

    # if specified, create the "ready indicator" file to indicate that the
    # backend is ready
    if args.ready_indicator:
        pathlib.Path(args.ready_indicator).touch()

    if cameras_enabled and args.camera_logfile:
        backend.wait_until_first_action()
        camera_logger.start()
        logging.info("Start camera logging")

    termination_reason = backend.wait_until_terminated()
    logging.debug("Backend termination reason: %d", termination_reason)

    if cameras_enabled:
        camera_backend.shutdown()

    # delete the ready indicator file to indicate that the backend has shut
    # down
    if args.ready_indicator:
        pathlib.Path(args.ready_indicator).unlink()

    if cameras_enabled and args.camera_logfile:
        logging.info(
            "Save recorded camera data to file %s", args.camera_logfile
        )
        camera_logger.stop_and_save(args.camera_logfile)

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
