#!/usr/bin/env python3
"""Run robot_interfaces Backend for pyBullet using multi-process robot data."""
import argparse
import logging
import math
import pathlib
import sys

import robot_interfaces
from trifinger_simulation import (
    collision_objects,
    finger_types_data,
)
import robot_fingers.pybullet_drivers as drivers


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--finger-type",
        choices=finger_types_data.get_valid_finger_types(),
        required=True,
        help="""Pass a valid finger type.""",
    )
    parser.add_argument(
        "--real-time-mode",
        "-r",
        action="store_true",
        help="""Run simulation in real time.  If not set, the simulation runs
            as fast as possible.
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
        "--max-number-of-actions",
        "-a",
        type=int,
        default=0,
        help="""Maximum numbers of actions that are processed.  After this the
            backend shuts down automatically.
        """,
    )
    parser.add_argument(
        "--robot-logfile",
        "-l",
        type=str,
        help="""Path to a file to which the robot data log is written.  If not
            specified, no log is generated.
        """,
    )
    parser.add_argument(
        "--camera-logfile",
        type=str,
        help="""Path to a file to which the camera data log is written.  If not
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
        "--add-cube",
        action="store_true",
        help="""Spawn a cube and run the object tracker backend.""",
    )
    parser.add_argument(
        "--cameras",
        action="store_true",
        help="""Run camera backend using rendered images.""",
    )
    parser.add_argument(
        "--visualize",
        "-v",
        action="store_true",
        help="Run pyBullet's GUI for visualization.",
    )
    args = parser.parse_args()

    # configure the logging module
    log_handler = logging.StreamHandler(sys.stdout)
    logging.basicConfig(
        format="[SIM_TRIFINGER_BACKEND %(levelname)s %(asctime)s] %(message)s",
        level=logging.DEBUG,
        handlers=[log_handler],
    )

    # select the correct types/functions based on which robot is used
    num_fingers = finger_types_data.get_number_of_fingers(args.finger_type)
    if num_fingers == 1:
        shared_memory_id = "finger"
        finger_types = robot_interfaces.finger
        create_backend = drivers.create_single_finger_backend
    elif num_fingers == 3:
        shared_memory_id = "trifinger"
        finger_types = robot_interfaces.trifinger
        create_backend = drivers.create_trifinger_backend

    # If max_number_of_actions is set, choose the history size of the time
    # series such that the whole episode fits in (+1 for the status message
    # containing the "limit exceeded" error).
    if args.max_number_of_actions:
        history_size = args.max_number_of_actions + 1
    else:
        history_size = 1000

    robot_data = finger_types.MultiProcessData(
        shared_memory_id, True, history_size=history_size
    )

    robot_logger = finger_types.Logger(robot_data)

    backend = create_backend(
        robot_data,
        args.real_time_mode,
        args.visualize,
        args.first_action_timeout,
        args.max_number_of_actions,
    )
    backend.initialize()

    #
    # Camera and Object Tracker Interface
    # Important:  These objects need to be created _after_ the simulation is
    # initialized (i.e. after the SimFinger instance is created).
    #
    if args.cameras and not args.add_cube:
        # If cameras are enabled but not the object, use the normal
        # PyBulletTriCameraDriver.
        from trifinger_cameras import tricamera

        camera_data = tricamera.MultiProcessData("tricamera", True, 10)
        camera_driver = tricamera.PyBulletTriCameraDriver()
        camera_backend = tricamera.Backend(camera_driver, camera_data)  # noqa

    elif args.add_cube:
        # If the cube is enabled, use the PyBulletTriCameraObjectTrackerDriver.
        # In case the cameras are not requested, disable rendering of the
        # images to save time.
        import trifinger_object_tracking.py_tricamera_types as tricamera

        # spawn a cube in the centre of the arena
        cube = collision_objects.Cuboid(
            position=[0.0, 0.0, 0.01],
            orientation=[0, 0, 0, 1],
            half_extents=[0.01, 0.04, 0.01],
            mass=0.016,
        )

        render_images = args.cameras

        camera_data = tricamera.MultiProcessData("tricamera", True, 10)
        camera_driver = tricamera.PyBulletTriCameraObjectTrackerDriver(
            cube, robot_data, render_images
        )
        camera_backend = tricamera.Backend(camera_driver, camera_data)  # noqa

    #
    # Camera/Object Logger
    #
    camera_logger = None
    if args.camera_logfile:
        try:
            camera_data
        except NameError:
            logging.critical("Cannot create camera log camera is not running.")
            return

        # TODO
        camera_fps = 10
        robot_rate_hz = 1000
        buffer_length_factor = 1.5
        episode_length_s = args.max_number_of_actions / robot_rate_hz
        # Compute camera log size based on number of robot actions plus some
        # safety buffer
        log_size = int(camera_fps * episode_length_s * buffer_length_factor)

        logging.info("Initialize camera logger with buffer size %d", log_size)
        camera_logger = tricamera.Logger(camera_data, log_size)

    # if specified, create the "ready indicator" file to indicate that the
    # backend is ready
    if args.ready_indicator:
        pathlib.Path(args.ready_indicator).touch()

    backend.wait_until_first_action()

    if camera_logger:
        camera_logger.start()
        logging.info("Start camera logging")

    backend.wait_until_terminated()

    # delete the ready indicator file to indicate that the backend has shut
    # down
    if args.ready_indicator:
        pathlib.Path(args.ready_indicator).unlink()

    if camera_logger:
        logging.info(
            "Save recorded camera data to file %s", args.camera_logfile
        )
        camera_logger.stop_and_save(args.camera_logfile)

    if camera_data:
        # stop the camera backend
        logging.info("Stop camera backend")
        camera_backend.shutdown()

    if args.robot_logfile:
        logging.info("Save robot data to file %s", args.robot_logfile)
        if args.max_number_of_actions:
            end_index = args.max_number_of_actions
        else:
            end_index = -1

        robot_logger.write_current_buffer_binary(
            args.robot_logfile, start_index=0, end_index=end_index
        )

    # cleanup stuff before the simulation (backend) is terminated
    if args.add_cube:
        del cube

    logging.info("Done.")


if __name__ == "__main__":
    main()
