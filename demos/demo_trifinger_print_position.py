#!/usr/bin/env python3
"""Send zero-torque commands to the robot and print joint positions."""
import robot_interfaces
import robot_fingers

if __name__ == "__main__":
    robot = robot_fingers.Robot(robot_interfaces.trifinger,
                              robot_fingers.create_trifinger_backend,
                              "trifinger.yml")
    robot.initialize()

    robot_fingers.demo_print_position(robot)
