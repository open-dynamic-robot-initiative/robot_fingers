#!/usr/bin/python3
"""Run single finger in "spring mode" for testing after assembly."""
import os
import curses
import numpy as np

import rospkg

from robot_interfaces import finger
import robot_fingers
import pybullet_fingers.drivers


class CursesGUI:
    def __init__(self, stdscr):
        self.stdscr = stdscr
        self.stdscr.nodelay(True)

    def update(self, observation, desired_action, applied_action):
        observation_data = np.vstack([
            observation.position, observation.velocity, observation.torque
        ]).T
        desired_action_data = np.vstack([desired_action.torque,
                                         desired_action.position]).T
        applied_action_data = np.vstack([applied_action.torque,
                                         applied_action.position]).T

        joint_names = ["Joint %d" % i for i in range(len(observation.torque))]

        self.stdscr.clear()

        # status line
        status_line = " Press 'q' to quit."
        max_rows, max_cols = self.stdscr.getmaxyx()
        self.stdscr.addstr(max_rows - 1, 0,
                           status_line.ljust(max_cols - 1, " "),
                           curses.A_STANDOUT)

        # header line
        line = 0
        self.stdscr.addstr(line, 0, "Single Finger Test Application",
                           curses.A_BOLD)
        line += 3

        line = self.draw_data_table(
            line,
            0,
            "OBSERVATION",
            joint_names,
            ["Position [rad]", "Velocity [rad/s]", "Torque [Nm]"],
            observation_data,
        )
        line += 2

        line = self.draw_data_table(
            line,
            0,
            "DESIRED ACTION",
            joint_names,
            ["Torque [Nm]", "Position [rad]"],
            desired_action_data,
        )
        line += 2

        line = self.draw_data_table(
            line,
            0,
            "APPLIED ACTION",
            joint_names,
            ["Torque [Nm]", "Position [rad]"],
            applied_action_data,
        )

        self.stdscr.refresh()

        # quit if user presses "q"
        c = self.stdscr.getch()
        return c != ord("q")

    def draw_data_table(
        self,
        start_line,
        start_column,
        title,
        row_header,
        column_header,
        data,
        column_width=None,
    ):
        rows, columns = data.shape
        assert len(row_header) == rows
        assert len(column_header) == columns

        column_margin = 3
        if column_width is None:
            column_width = max((len(h) for h in column_header)) + column_margin

        first_column_width = max((len(h) for h in row_header)) + column_margin

        total_width = first_column_width + column_width * columns

        line = start_line

        self.stdscr.addstr(line, start_column, title, curses.A_BOLD)
        line += 1
        self.stdscr.addstr(line, start_column, "━" * total_width)
        line += 1

        # column headers
        column = start_column + first_column_width
        for header in column_header:
            self.stdscr.addstr(line, column, header, curses.A_BOLD)
            column += column_width
        line += 1
        self.stdscr.addstr(line, start_column, "─" * total_width)
        line += 1

        # data rows
        for i, header in enumerate(row_header):
            self.stdscr.addstr(line, start_column, header, curses.A_BOLD)
            column = start_column + first_column_width

            for j in range(columns):
                self.stdscr.addstr(line, column, "{: .3f}".format(data[i, j]))
                column += column_width

            line += 1

        self.stdscr.addstr(line, start_column, "━" * total_width)
        line += 1

        return line


def loop(stdscr, frontend):
    gui = CursesGUI(stdscr)
    okay = True

    # get current position
    t = frontend.append_desired_action(finger.Action())
    target_position = frontend.get_observation(t).position

    while okay:
        desired_action = finger.Action(position=target_position)
        t = frontend.append_desired_action(desired_action)
        obs = frontend.get_observation(t)
        applied_action = frontend.get_applied_action(t)
        okay = gui.update(obs, desired_action, applied_action)


def main():
    USE_SIMULATION = True

    robot_data = finger.SingleProcessData()
    if USE_SIMULATION:
        backend = pybullet_fingers.drivers.create_single_finger_backend(
            robot_data, real_time_mode=True, visualize=True
        )
    else:
        # load the default config file
        config_file_path = os.path.join(
            rospkg.RosPack().get_path("robot_fingers"),
            "config",
            "single_finger_assembly_test.yml",
        )
        backend = robot_fingers.create_one_joint_backend(
            robot_data, config_file_path
        )

    frontend = finger.Frontend(robot_data)

    backend.initialize()

    curses.wrapper(lambda stdscr: loop(stdscr, frontend))


if __name__ == "__main__":
    main()
