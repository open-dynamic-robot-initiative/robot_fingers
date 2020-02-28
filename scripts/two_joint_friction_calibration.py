#!/usr/bin/python3
"""Curses-based application to aid calibrating joint friction.

Rotates the joints of the TwoJoint robot with constant velocity using the
position controller (remove endstops of the robot to enable rotation!).
Prints averaged torque/current and velocity measurements from which the
friction can be estimated.

See `--help` for options.
"""

import argparse
import copy
import curses
import os
import time
import numpy as np
import rospkg

from robot_interfaces import two_joint
import robot_fingers


N_JOINTS = 2

CURRENT_TO_TORQUE_FACTOR = 0.02 * 9


class AverageBuffer:

    def __init__(self, size):
        self.size = size
        self.buffer = []

    def append(self, entry):
        self.buffer.append(entry)
        if len(self.buffer) > self.size:
            self.buffer = self.buffer[1:]

    def mean(self):
        return np.array(self.buffer).mean(axis=0)


def run_application(stdscr, robot, velocity_radps, buffer_size):

    stdscr.nodelay(True)

    enabled = False

    desired_velocity_radps = velocity_radps
    step_duration_s = 0.001

    max_following_error_rad = np.pi / 2

    step_size_rad = desired_velocity_radps * step_duration_s

    velocity_buffer = AverageBuffer(buffer_size)
    measured_torque_buffer = AverageBuffer(buffer_size)
    applied_torque_buffer = AverageBuffer(buffer_size)

    # start by sending a zero torque command
    t = robot.append_desired_action(two_joint.Action())
    obs = robot.get_observation(t)
    # start position profile at current position
    desired_position = copy.copy(obs.position)

    desired_torque = np.zeros(N_JOINTS)
    action = two_joint.Action(position=desired_position)
    last_update = 0
    while True:
        t = robot.append_desired_action(action)
        obs = robot.get_observation(t)
        applied_action = robot.get_applied_action(t)

        if enabled:
            # TODO is the python loop fast enough?
            # FIXME handle overflow!  or maybe simply stop if following error
            # is too high
            desired_position = desired_position + step_size_rad

            following_error = desired_position - obs.position
            if np.any(np.abs(following_error) > max_following_error_rad):
                raise RuntimeError("Following error too high: %s" %
                                   following_error)

            action = two_joint.Action(position=desired_position)
        else:
            action = two_joint.Action()
            # set desired position to current one to avoid jumps when enabling
            desired_position = obs.position

        velocity_buffer.append(obs.velocity)
        measured_torque_buffer.append(obs.torque)
        applied_torque_buffer.append(applied_action.torque)

        now = time.time()
        if now - last_update > 0.1:
            last_update = now

            # generate the output
            stdscr.clear()
            line = 0
            # help line at the bottom
            stdscr.addstr(curses.LINES - 1, 0,
                          " s: start/stop motors | q: quit ",
                          curses.A_STANDOUT)

            stdscr.addstr(line, 0, "Status:", curses.A_BOLD)
            stdscr.addstr(line, len("Status:") + 1,
                          "RUNNING" if enabled else "STOPPED")

            line += 2
            stdscr.addstr(line, 0, "Commanded Torque:", curses.A_BOLD)
            for i, p in enumerate(applied_torque_buffer.mean()):
                line += 1
                stdscr.addstr(line, 4, "Joint {}: {: .3f} Nm / {: .3f} A".format(
                    i, p, p / CURRENT_TO_TORQUE_FACTOR))

            line += 2
            stdscr.addstr(line, 0, "Measured Torque:", curses.A_BOLD)
            for i, p in enumerate(measured_torque_buffer.mean()):
                line += 1
                stdscr.addstr(line, 4, "Joint {}: {: .3f} Nm / {: .3f} A".format(
                    i, p, p / CURRENT_TO_TORQUE_FACTOR))

            line += 2
            stdscr.addstr(line, 0, "Velocity:", curses.A_BOLD)
            for i, p in enumerate(velocity_buffer.mean()):
                line += 1
                stdscr.addstr(line, 4, "Joint {}: {:.3f}".format(i, p))

            line += 2
            stdscr.addstr(line, 0, "Position (actual / desired):", curses.A_BOLD)
            for i, (p, d) in enumerate(zip(obs.position, desired_position)):
                line += 1
                stdscr.addstr(line, 4, "Joint {}: {:.3f} / {:.3f}".format(i, p,
                                                                          d))

            stdscr.refresh()

        c = stdscr.getch()
        if c == ord("q"):
            return
        if c == ord("s"):
            enabled = not enabled
        else:
            pass


def main():
    argparser = argparse.ArgumentParser()
    argparser.add_argument("--velocity", type=float, default=np.pi/2,
                           help="Velocity at which joints are moving [rad/s].")
    argparser.add_argument("--average-window", type=int, default=50,
                           help="Number of values used for averaging.")
    args = argparser.parse_args()


    # load the default config file
    config_file_path = os.path.join(
        rospkg.RosPack().get_path("robot_fingers"), "config",
        "twojoint_friction_calibration.yml")

    robot_data = two_joint.Data()
    robot_backend = robot_fingers.create_two_joint_backend(robot_data,
                                                         config_file_path)
    robot_frontend = two_joint.Frontend(robot_data)

    robot_backend.initialize()

    curses.wrapper(lambda stdscr: run_application(
        stdscr, robot_frontend, args.velocity, args.average_window))


if __name__ == "__main__":
    main()
