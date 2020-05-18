#!/usr/bin/env python3
"""Forward kinematic example using pinocchio

Sends zero-torque commands to the robot and prints finger tip position.
"""
import os
import numpy as np
import rospkg
import pinocchio
import robot_interfaces
import robot_fingers

if __name__ == "__main__":
    urdf_pkg_path = rospkg.RosPack().get_path("robot_properties_manipulator")
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

    action = robot.Action()
    while True:
        t = robot.frontend.append_desired_action(action)
        joint_positions = robot.frontend.get_observation(t).position

        # compute the forward kinematics
        pinocchio.framesForwardKinematics(model, data, joint_positions)

        # get the position of the tip link
        pos = data.oMf[tip_link_id].translation

        # convert from np.matrix to a flat array (for easy printing)
        pos = np.asarray(pos).reshape(-1)
        n_joints = len(pos)
        format_string = "\r" + ", ".join(["{: 6.3f}"] * n_joints)
        print(format_string.format(*pos), end="")
