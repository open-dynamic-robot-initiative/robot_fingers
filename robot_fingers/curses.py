"""Tools for creating simple curses interfaces."""

import curses


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
