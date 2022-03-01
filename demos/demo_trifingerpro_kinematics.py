#!/usr/bin/env python3
"""Demo computing forward and inverse kinematics for the TriFingerPro robot.

Uses forward and inverse kinematics to move the finger tips synchronously back
and forth on the y-axis.

For the computation the `Kinematics` class from the trifinger_simulation
package is used, so this package needs to be installed.
For complete documentation of the `Kinematics` class see
https://open-dynamic-robot-initiative.github.io/trifinger_simulation/api/pinocchio_utils.html
"""
import argparse
import copy
import os

import numpy as np
from ament_index_python.packages import get_package_share_directory

import trifinger_simulation.finger_types_data
import trifinger_simulation.pinocchio_utils
import robot_interfaces
import robot_fingers


def init_kinematics():
    """Initialise the kinematics calculator for TriFingerPro."""
    robot_properties_path = get_package_share_directory(
        "robot_properties_fingers"
    )
    urdf_file = trifinger_simulation.finger_types_data.get_finger_urdf(
        "trifingerpro"
    )
    finger_urdf_path = os.path.join(robot_properties_path, "urdf", urdf_file)
    kinematics = trifinger_simulation.pinocchio_utils.Kinematics(
        finger_urdf_path,
        ["finger_tip_link_0", "finger_tip_link_120", "finger_tip_link_240"],
    )

    return kinematics


def main():
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        "--single-process",
        action="store_true",
        help="Run back- and front-end in the same process.",
    )
    args = argparser.parse_args()

    if args.single_process:
        robot = robot_fingers.Robot.create_by_name("trifingerpro")
        robot.initialize()
        frontend = robot.frontend
    else:
        robot_data = robot_interfaces.trifinger.MultiProcessData(
            "trifinger", False
        )
        frontend = robot_interfaces.trifinger.Frontend(robot_data)

    kinematics = init_kinematics()

    # initial angular joint positions in radian
    angular_joint_positions = np.array([0, 0.9, -1.7] * 3)

    # Use forward kinematics to get the initial finger tip positions.
    # The returned value is a list of (x, y, z)-positions of the three finger
    initial_cartesian_tip_positions = kinematics.forward_kinematics(
        angular_joint_positions
    )
    for i, pos in enumerate(initial_cartesian_tip_positions):
        print("Initial position of finger tip {}: {}".format(i, pos))

    # move the tips back and forth on the y-axis
    cartesian_tip_positions = copy.deepcopy(initial_cartesian_tip_positions)
    distance = 0.05
    n_steps = 1000
    t = 0
    while True:
        # apply action and get observation
        finger_action = robot_interfaces.trifinger.Action(
            position=angular_joint_positions
        )
        t = frontend.append_desired_action(finger_action)
        observation = frontend.get_observation(t)

        # update desired tip positions
        for i in range(len(initial_cartesian_tip_positions)):
            dy = distance * np.sin(t * np.pi / n_steps)
            cartesian_tip_positions[i][1] = (
                initial_cartesian_tip_positions[i][1] + dy
            )

        # use inverse kinematics to get the corresponding joint angles
        angular_joint_positions, err = kinematics.inverse_kinematics(
            cartesian_tip_positions, observation.position
        )
        print("IK error:", [round(np.linalg.norm(e), 4) for e in err])


if __name__ == "__main__":
    main()
