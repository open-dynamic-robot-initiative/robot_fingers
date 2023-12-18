#!/usr/bin/env python3
"""Run robot_interfaces Backend for PyBullet using multi-process robot data.

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
        "--object",
        type=str,
        choices=["cube", "dice", "none"],
        default="cube",
        metavar="OBJECT_TYPE",
        help="Which type of object to use (if any).",
    )
    parser.add_argument(
        "--cameras",
        action="store_true",
        help="""Run camera backend providing observations.  Note that by
            default images in the observations are not rendered for performance
            reasons.  To enable rendering set --render-images.  If
            --object=cube is set, TriCameraObjectObservations are provided
            (which include the object pose), otherwise normal
            TriCameraObservations are used.""",
    )
    parser.add_argument(
        "--render-images",
        action="store_true",
        help="""Render camera images.  If cameras are enabled without this
            being set, camera observations are provided but the actual images
            in the observations will be uninitialised.  Rendering is rather
            slow, so don't enable this if you need the simulation to run in
            more or less real-time.""",
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

    robot_data = robot_interfaces.trifinger.MultiProcessData("trifinger", False)

    backend = drivers.create_trifinger_backend(
        robot_data,
        args.real_time_mode,
        args.visualize,
        args.first_action_timeout,
        args.max_number_of_actions,
    )
    backend.initialize()

    #
    # Add object to simulation
    #
    if args.object == "cube":
        # spawn a cube in the centre of the arena
        cube = collision_objects.ColoredCubeV2(
            position=[0.0, 0.0, 0.0325],
            orientation=[0, 0, 0, 1],
        )
    elif args.object == "dice":
        from trifinger_simulation.tasks import rearrange_dice
        from trifinger_simulation.sim_finger import int_to_rgba

        die_mass = 0.012
        # use a random goal for initial positions
        initial_positions = rearrange_dice.sample_goal()
        dice = [
            collision_objects.Cube(
                position=pos,
                half_width=rearrange_dice.DIE_WIDTH / 2,
                mass=die_mass,
                color_rgba=int_to_rgba(0x0A7DCF),
            )
            for pos in initial_positions
        ]

    #
    # Camera and Object Tracker Interface
    # Important:  These objects need to be created _after_ the simulation is
    # initialized (i.e. after the SimFinger instance is created).
    #
    if args.cameras:
        if args.object == "cube":
            # If the cube is enabled, use the
            # PyBulletTriCameraObjectTrackerDriver.
            import trifinger_object_tracking.py_tricamera_types as tricamera

            camera_data = tricamera.MultiProcessData("tricamera", False)
            camera_driver = tricamera.PyBulletTriCameraObjectTrackerDriver(
                cube, robot_data, args.render_images
            )
            camera_backend = tricamera.Backend(camera_driver, camera_data)

        else:
            # If cameras are enabled but not the object, use the normal
            # PyBulletTriCameraDriver.
            from trifinger_cameras import tricamera

            camera_data = tricamera.MultiProcessData("tricamera", False)
            camera_driver = tricamera.PyBulletTriCameraDriver(
                robot_data, render_images=args.render_images
            )
            camera_backend = tricamera.Backend(camera_driver, camera_data)

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

    if args.cameras:
        # stop the camera backend
        logging.info("Stop camera backend")
        camera_backend.shutdown()

    termination_reason = backend.get_termination_reason()
    logger.debug("Backend termination reason: %d" % termination_reason)

    # cleanup stuff before the simulation (backend) is terminated
    if args.object == "cube":
        del cube
    elif args.object == "dice":
        del dice[:]

    rclpy.shutdown()
    if termination_reason < 0:
        # negate code as exit codes should be positive
        return -termination_reason
    else:
        return 0


if __name__ == "__main__":
    sys.exit(main())
