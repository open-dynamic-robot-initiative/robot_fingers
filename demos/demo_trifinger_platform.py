#!/usr/bin/env python3
"""Move TriFingerPro on simple sine position profile."""
import argparse

import numpy as np

import robot_interfaces
import robot_fingers


def get_target_joint_positions(t: int) -> np.ndarray:
    """Compute target joint positions for time step t using simple sine profile."""
    INITIAL_POSITION_RAD = np.array([0.0, 0.9, -1.7] * 3)
    AMPLITUDE_RAD = np.deg2rad([10, 20, 20] * 3)
    FREQUENCY_HZ = 1.0

    ROBOT_CONTROL_FREQ_HZ = 1000

    t_s = t / ROBOT_CONTROL_FREQ_HZ
    target = INITIAL_POSITION_RAD + AMPLITUDE_RAD * np.sin(
        2 * np.pi * FREQUENCY_HZ * t_s
    )

    return target


def parse_arguments() -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--with-object",
        "-o",
        action="store_true",
        help="Set if backend provides object tracking data.",
    )

    return parser.parse_args()



def main() -> None:  # noqa[D103]
    args = parse_arguments()

    if args.with_object:
        robot = robot_fingers.TriFingerPlatformWithObjectFrontend()
    else:
        robot = robot_fingers.TriFingerPlatformFrontend()

    t = 0
    while True:
        action = robot_interfaces.trifinger.Action(
            position=get_target_joint_positions(t)
        )
        t = robot.append_desired_action(action)

        robot_observation = robot.get_robot_observation(t)
        camera_observation = robot.get_camera_observation(t)

        if t % 100 == 0:
            print(f"[{t}] target joint positions: {action.position}")
            print(f"[{t}] actual joint positions: {robot_observation.position}")

            print(f"[{t}] Image shape: {camera_observation.cameras[0].image.shape}")

            if args.with_object:
                object_pos = camera_observation.filtered_object_pose.position
                print(f"[{t}] Object position: {object_pos}")

            print()


if __name__ == "__main__":
    main()
