#!/usr/bin/python3
"""Run single finger in position control mode and print all robot data."""
import os
import curses
import numpy as np

from ament_index_python.packages import get_package_share_directory

from robot_interfaces import finger
import robot_fingers


class CursesGUI:
    """Displays robot data using curses."""

    def __init__(self, win):
        """Initialize.

        Args:
            win: Curses window.
        """
        self.win = win
        self.win.nodelay(True)

    def update(self, observation, desired_action, applied_action, status):
        """Update the displayed robot data."""
        # arrange data in arrays
        motor_observation_data = np.vstack(
            [observation.position, observation.velocity, observation.torque]
        ).T
        desired_action_data = np.vstack(
            [desired_action.torque, desired_action.position]
        ).T
        applied_action_data = np.vstack(
            [applied_action.torque, applied_action.position]
        ).T

        joint_names = ["Joint %d" % i for i in range(len(observation.torque))]

        self.win.erase()

        try:

            # status line
            status_line = " Press 'q' to quit."
            max_rows, max_cols = self.win.getmaxyx()
            self.win.insstr(
                max_rows - 1,
                0,
                status_line.ljust(max_cols, " "),
                curses.A_STANDOUT,
            )

            # header line
            line = 0
            self.win.addstr(
                line, 0, "Single Finger Test Application", curses.A_BOLD
            )
            line += 3

            # draw the data tables
            line = self.draw_data_table(
                line,
                0,
                "OBSERVATION",
                joint_names,
                ["Position [rad]", "Velocity [rad/s]", "Torque [Nm]"],
                motor_observation_data,
            )
            self.win.addstr(
                line, 0, "Tip Force: {}".format(observation.tip_force)
            )
            line += 1
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
            line += 2

            # status does not fit well in a table, so list values manually here
            self.win.addstr(line, 0, "STATUS", curses.A_BOLD)
            line += 1
            self.win.addstr(line, 0, "━" * 40)
            line += 1
            self.win.addstr(
                line,
                0,
                "Action Repetitions: {}".format(status.action_repetitions),
            )
            line += 1
            self.win.addstr(
                line, 0, "Error Status: {}".format(status.error_status)
            )
            line += 1
            self.win.addstr(
                line, 0, "Error Message: {}".format(status.get_error_message())
            )
            line += 1
            self.win.addstr(line, 0, "━" * 40)

            self.win.refresh()

        except curses.error as e:
            raise RuntimeError(
                "GUI rendering error.  Try increasing the terminal window."
            )

        # quit if user presses "q"
        c = self.win.getch()
        return c != ord("q")

    def display_error(self, message):
        """Display error message and wait until user presses a key.

        Args:
            message:  The error message that is displayed.
        """
        self.win.nodelay(False)
        self.win.clear()

        self.win.addstr(1, 0, "ERROR:", curses.A_BOLD)
        self.win.addstr(3, 4, message)
        self.win.addstr(5, 0, "Press any key to exit.")
        self.win.refresh()

        self.win.getch()

    def draw_data_table(
        self,
        y,
        x,
        title,
        row_headers,
        column_headers,
        data,
        column_width=None,
    ):
        """Draw a table with data.

        Args:
            y:  y-coordinate of the top-left corner of the table.
            x:  x-coordinate of the top-left corner of the table.
            title:  Title of the table (printed above).
            row_headers:  List of row headers, one for each row in the data.
            column_headers:  List of column headers, one for each column in the
                data.
            data:  Data-array.  Shape has to match with the lengths of
                row_headers and column_headers.
            column_width:  Width of the columns.  If None, the width is
                determined based on the length of the longest column header.
        """
        rows, columns = data.shape
        assert len(row_headers) == rows
        assert len(column_headers) == columns

        column_margin = 3
        if column_width is None:
            column_width = (
                max((len(h) for h in column_headers)) + column_margin
            )

        first_column_width = max((len(h) for h in row_headers)) + column_margin

        total_width = first_column_width + column_width * columns

        line = y

        self.win.addstr(line, x, title, curses.A_BOLD)
        line += 1
        self.win.addstr(line, x, "━" * total_width)
        line += 1

        # column headers
        column = x + first_column_width
        for header in column_headers:
            self.win.addstr(line, column, header, curses.A_BOLD)
            column += column_width
        line += 1
        self.win.addstr(line, x, "─" * total_width)
        line += 1

        # data rows
        for i, header in enumerate(row_headers):
            self.win.addstr(line, x, header, curses.A_BOLD)
            column = x + first_column_width

            for j in range(columns):
                self.win.addstr(line, column, "{: .3f}".format(data[i, j]))
                column += column_width

            line += 1

        self.win.addstr(line, x, "━" * total_width)
        line += 1

        return line


def loop(win, frontend):
    gui = CursesGUI(win)
    okay = True

    try:
        # get current position
        t = frontend.append_desired_action(finger.Action())
        target_position = frontend.get_observation(t).position

        while okay:
            desired_action = finger.Action(position=target_position)
            t = frontend.append_desired_action(desired_action)
            obs = frontend.get_observation(t)
            applied_action = frontend.get_applied_action(t)
            status = frontend.get_status(t)

            okay = gui.update(obs, desired_action, applied_action, status)
    except Exception as e:
        gui.display_error(str(e))


def main():
    USE_SIMULATION = False

    robot_data = finger.SingleProcessData()
    if USE_SIMULATION:
        import trifinger_simulation.drivers

        backend = trifinger_simulation.drivers.create_single_finger_backend(
            robot_data, real_time_mode=True, visualize=True
        )
    else:
        # load the default config file
        config_file_path = os.path.join(
            get_package_share_directory("robot_fingers"),
            "config",
            "single_finger_test.yml",
        )
        backend = robot_fingers.create_real_finger_backend(
            robot_data, config_file_path
        )

    frontend = finger.Frontend(robot_data)
    backend.initialize()

    curses.wrapper(lambda stdscr: loop(stdscr, frontend))


if __name__ == "__main__":
    main()
