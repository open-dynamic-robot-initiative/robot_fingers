#!/usr/bin/env python3
"""Run robot_interfaces Backend for pyBullet using multi-process robot data.

This is intended to be used together with trifinger_data_backend.py and can
serve as a replacement for the real robot backend.
"""
import argparse
import logging
import math
import sys

import rclpy

import robot_interfaces
from trifinger_simulation import collision_objects
import robot_fingers.pybullet_drivers as drivers
from robot_fingers.ros import NotificationNode


def main():
    parser = argparse.ArgumentParser(description=__doc__)
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

    rclpy.init()
    node = NotificationNode("trifinger_backend")
    logger = node.get_logger()

    robot_data = robot_interfaces.trifinger.MultiProcessData(
        "trifinger", False
    )

    backend = drivers.create_trifinger_backend(
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

        camera_data = tricamera.MultiProcessData("tricamera", False)
        camera_driver = tricamera.PyBulletTriCameraDriver()
        camera_backend = tricamera.Backend(camera_driver, camera_data)  # noqa

    elif args.add_cube:
        # If the cube is enabled, use the PyBulletTriCameraObjectTrackerDriver.
        # In case the cameras are not requested, disable rendering of the
        # images to save time.
        import trifinger_object_tracking.py_tricamera_types as tricamera

        # spawn a cube in the centre of the arena
        cube = collision_objects.ColoredCubeV2(
            position=[0.0, 0.0, 0.0325],
            orientation=[0, 0, 0, 1],
        )

        render_images = args.cameras

        camera_data = tricamera.MultiProcessData("tricamera", False)
        camera_driver = tricamera.PyBulletTriCameraObjectTrackerDriver(
            cube, robot_data, render_images
        )
        camera_backend = tricamera.Backend(camera_driver, camera_data)  # noqa

    logger.info("Robot Simulation backend is ready")

    # send ready signal
    node.publish_status("READY")

    # wait until backend terminates or shutdown request is received
    while backend.is_running():
        rclpy.spin_once(node, timeout_sec=1)
        if node.shutdown_requested:
            backend.request_shutdown()
            backend.wait_until_terminated()
            break

    if camera_data:
        # stop the camera backend
        logging.info("Stop camera backend")
        camera_backend.shutdown()

    termination_reason = backend.get_termination_reason()
    logger.debug("Backend termination reason: %d" % termination_reason)

    # cleanup stuff before the simulation (backend) is terminated
    if args.add_cube:
        del cube

    rclpy.shutdown()
    if termination_reason < 0:
        # negate code as exit codes should be positive
        return -termination_reason
    else:
        return 0


if __name__ == "__main__":
    sys.exit(main())
