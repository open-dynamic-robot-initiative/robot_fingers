#!/usr/bin/env python3
"""Send zero-torque commands to the robot and print joint positions."""
import robot_interfaces
import robot_fingers

if __name__ == "__main__":
    robot = robot_fingers.Robot(robot_interfaces.two_joint,
                              robot_fingers.create_two_joint_backend,
                              "twojoint.yml")
    robot.initialize()

    robot_fingers.demo_print_position(robot)
