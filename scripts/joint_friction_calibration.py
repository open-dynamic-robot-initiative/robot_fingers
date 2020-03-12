#!/usr/bin/python3
"""Curses-based application to aid calibrating joint friction.

Rotates the joints of the OneJoint robot with constant velocity using a simple
PI velocity controller.
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

from robot_interfaces import one_joint
import robot_fingers


N_JOINTS = 1

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

    velocity_buffer = AverageBuffer(buffer_size)
    measured_torque_buffer = AverageBuffer(buffer_size)
    applied_torque_buffer = AverageBuffer(buffer_size)

    # PI controller parameters
    kp = 0.1
    ki = 0.2
    max_error_integal = 10
    error_integral = np.zeros(N_JOINTS)
    desired_velocity_radps = np.ones(N_JOINTS) * desired_velocity_radps

    # start by sending a zero torque command
    action = one_joint.Action()
    t = robot.append_desired_action(action)
    obs = robot.get_observation(t)

    last_update = 0
    while True:
        t = robot.append_desired_action(action)
        obs = robot.get_observation(t)
        applied_action = robot.get_applied_action(t)

        if enabled:
            # run simple PI velocity controller
            velocity_error = desired_velocity_radps - obs.velocity
            error_integral += velocity_error * step_duration_s
            error_integral = np.clip(error_integral,
                                     -max_error_integal,
                                     +max_error_integal)
            torque = kp * velocity_error + ki * error_integral

            action = one_joint.Action(torque=torque)
        else:
            action = one_joint.Action()

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

            stdscr.addstr(line, 0,
                          "Values are averaged over {:.3f} seconds".format(
                              buffer_size * step_duration_s))

            line += 2
            stdscr.addstr(line, 0, "Status:", curses.A_BOLD)
            stdscr.addstr(line, len("Status:") + 1,
                          "RUNNING" if enabled else "STOPPED")

            line += 2
            stdscr.addstr(line, 0, "Commanded Torque:", curses.A_BOLD)
            for i, p in enumerate(applied_torque_buffer.mean()):
                line += 1
                stdscr.addstr(line, 4,
                              "Joint {}: {: .3f} Nm / {: .3f} A".format(
                                  i, p, p / CURRENT_TO_TORQUE_FACTOR))

            line += 2
            stdscr.addstr(line, 0, "Measured Torque:", curses.A_BOLD)
            for i, p in enumerate(measured_torque_buffer.mean()):
                line += 1
                stdscr.addstr(line, 4,
                              "Joint {}: {: .3f} Nm / {: .3f} A".format(
                                  i, p, p / CURRENT_TO_TORQUE_FACTOR))

            line += 2
            stdscr.addstr(line, 0, "Velocity (actual / desired):",
                          curses.A_BOLD)
            for i, (p, d) in enumerate(zip(velocity_buffer.mean(),
                                           desired_velocity_radps)):
                line += 1
                stdscr.addstr(line, 4, "Joint {}: {:.3f} / {:3f}".format(
                    i, p, d))

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
    argparser.add_argument("--average-window", type=int, default=1000,
                           help="Number of values used for averaging.")
    args = argparser.parse_args()


    # load the default config file
    config_file_path = os.path.join(
        rospkg.RosPack().get_path("robot_fingers"), "config",
        "onejoint_friction_calibration.yml")

    robot_data = one_joint.SingleProcessData()
    robot_backend = robot_fingers.create_one_joint_backend(robot_data,
                                                           config_file_path)
    robot_frontend = one_joint.Frontend(robot_data)

    robot_backend.initialize()

    curses.wrapper(lambda stdscr: run_application(
        stdscr, robot_frontend, args.velocity, args.average_window))


if __name__ == "__main__":
    main()
