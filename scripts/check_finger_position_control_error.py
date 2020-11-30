#!/usr/bin/env python3
"""Send random position goals and show error of end-effector position.

Samples random joint positions and uses them as targets for position actions.
For each goal, compute the error in the end-effector position.
"""
import os
import numpy as np
from ament_index_python.packages import get_package_share_directory
import pinocchio
import robot_interfaces
import robot_fingers


def get_random_position():
    """Generate a random position within a save range."""
    position_min = np.array([-1, -1, -2])
    position_max = np.array([1, 1, 2])

    position_range = position_max - position_min

    return position_min + np.random.rand(3) * position_range


def forward_kinematics(joint_positions):
    """compute the forward kinematics"""
    pinocchio.framesForwardKinematics(model, data, joint_positions)

    # get the position of the tip link
    pos = data.oMf[tip_link_id].translation

    # convert from np.matrix to a flat array (for easy printing)
    pos = np.asarray(pos).reshape(-1)

    return pos


if __name__ == "__main__":
    urdf_pkg_path = get_package_share_directory("robot_properties_fingers")
    urdf_path = os.path.join(urdf_pkg_path, "urdf", "finger.urdf")

    model = pinocchio.buildModelFromUrdf(urdf_path)
    data = model.createData()
    tip_link_id = model.getFrameId("finger_tip_link")

    robot = robot_fingers.Robot(
        robot_interfaces.finger,
        robot_fingers.create_real_finger_backend,
        "finger.yml",
    )
    robot.initialize()

    while True:
        desired_position = get_random_position()
        for _ in range(3000):
            # Appends a torque command ("action") to the action queue.
            # Returns the time step at which the action is going to be
            # executed.
            action = robot_interfaces.finger.Action(position=desired_position)
            t = robot.frontend.append_desired_action(action)
            robot.frontend.wait_until_timeindex(t)

        desired_tip_pos = forward_kinematics(desired_position)
        actual_tip_pos = forward_kinematics(
            robot.frontend.get_observation(t).position
        )
        error = desired_tip_pos - actual_tip_pos
        print("Position Error: {:.3}  {}".format(np.linalg.norm(error), error))
