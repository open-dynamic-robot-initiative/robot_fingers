#!/usr/bin/env python3
"""Record data while the user is manually moving the robot."""
import argparse
import curses

import robot_fingers


class SimpleCursesGUI:
    """Wrapper around curses to manage simple generic GUIs.

    A very simple curses interface with a title at the top, a static status
    line at the bottom and some arbitrary text in between that can be updated.
    """

    def __init__(self, win, title, status_line=None):
        """Initialize.

        Args:
            win: Curses window.
        """
        self.win = win
        self.win.nodelay(True)
        self.title = title
        self.status_line = status_line

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

    def update_screen(self, lines):
        """Update the screen with the given lines of text.

        Args:
            lines (list):  List of strings.  They are drawn in separate lines.
        """
        try:
            self.win.erase()

            self.win.addstr(0, 0, self.title, curses.A_BOLD)

            # status line
            if self.status_line:
                max_rows, max_cols = self.win.getmaxyx()
                self.win.insstr(
                    max_rows - 1,
                    0,
                    self.status_line.ljust(max_cols, " "),
                    curses.A_STANDOUT,
                )

            for i, line in enumerate(lines):
                self.win.addstr(2 + i, 0, line)

            self.win.refresh()
        except curses.error:
            raise RuntimeError(
                "GUI rendering error.  Try increasing the terminal window."
            )

    def get_pressed_key(self):
        """Get key pressed by the user."""
        return self.win.getch()


def loop(win, args):
    title = "Demonstration Recorder"
    status_line = " q: quit | space: start/stop recording"

    robot = robot_fingers.Robot.create_by_name(
        args.robot_type, logger_buffer_size=300000
    )
    robot.initialize()

    gui = SimpleCursesGUI(win, title, status_line)

    is_recording = False
    try:
        while True:
            t = robot.frontend.append_desired_action(robot.Action())
            robot.frontend.wait_until_timeindex(t)

            if is_recording:
                gui.update_screen(["Recording...", "Press [space] to stop."])
            else:
                gui.update_screen(["Press [space] to start recording."])

            pressed_key = gui.get_pressed_key()
            if pressed_key == ord("q"):
                # quit
                return
            elif pressed_key == ord(" "):
                if is_recording:
                    robot.logger.stop_and_save(
                        args.outfile, robot.logger.Format.CSV
                    )
                else:
                    robot.logger.start()

                is_recording = not is_recording

    except Exception as e:
        gui.display_error(str(e))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "robot_type",
        choices=robot_fingers.Robot.get_supported_robots(),
        help="Name of the robot.",
    )
    parser.add_argument(
        "outfile", type=str, help="Output file for the recorded data."
    )
    args = parser.parse_args()

    curses.wrapper(lambda stdscr: loop(stdscr, args))


if __name__ == "__main__":
    main()
