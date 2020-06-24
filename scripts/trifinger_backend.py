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
    args = parser.parse_args()

    # Use the default config file from the robot_fingers package
    config_file_path = os.path.join(
        rospkg.RosPack().get_path("robot_fingers"), "config", "trifinger.yml"
    )

    # Storage for all observations, actions, etc.
    robot_data = robot_interfaces.trifinger.MultiProcessData("trifinger", True)

    # The backend sends actions from the data to the robot and writes
    # observations from the robot to the data.
    backend = robot_fingers.create_trifinger_backend(
        robot_data, config_file_path
    )

    # Initializes the robot (e.g. performs homing).
    backend.initialize()

    if args.cameras:
        import trifinger_cameras

        camera_data = trifinger_cameras.tricamera.MultiProcessData(
            "tricamera", True, 100
        )
        camera_driver = trifinger_cameras.tricamera.TriCameraDriver(
            "camera60", "camera180", "camera300"
        )
        camera_backend = trifinger_cameras.tricamera.Backend(
            camera_driver, camera_data
        )

    if args.fake_object_tracker:
        import trifinger_object_tracking.py_object_tracker as object_tracker
        object_tracker_data = object_tracker.Data("object_tracker", True)
        object_tracker_backend = object_tracker.FakeBackend(
            object_tracker_data
        )


    while backend.wait_until_terminated():
        pass


if __name__ == "__main__":
    main()
