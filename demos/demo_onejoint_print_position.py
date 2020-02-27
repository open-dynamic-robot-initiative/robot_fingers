#!/usr/bin/env python3
"""Send zero-torque commands to the robot and print joint positions."""
import robot_interfaces
import blmc_robots

if __name__ == "__main__":
    robot = blmc_robots.Robot(robot_interfaces.one_joint,
                              blmc_robots.create_one_joint_backend,
                              "onejoint.yml")
    robot.initialize()

    blmc_robots.demo_print_position(robot)
