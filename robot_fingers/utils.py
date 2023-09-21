"""Utility classes/functions for the robot_fingers package."""
import time
import typing as t

import numpy as np
import numpy.typing as npt


class TimePrinter:
    """Regularly print the current date/time and the passed duration in hours."""

    def __init__(self, interval_s: float = 3600.0) -> None:
        """Initialise.

        Args:
            interval_s:  Print interval in seconds.
        """
        #: Print interval in seconds
        self.interval_s = interval_s
        #: Timestamp when the printer started
        self.start_time = time.time()
        self.start_time_str = time.strftime("%F %H:%M")

        self._last_time_print = 0.0

    def update(self) -> None:
        """Print time if the interval_s has passed since the last call."""
        now = time.time()
        if now - self._last_time_print > self.interval_s:
            time_str = time.strftime("%F %H:%M")
            duration_h = round((now - self.start_time) / 3600)
            print(
                "{} ({} h since {})".format(time_str, duration_h, self.start_time_str)
            )
            self._last_time_print = now


def min_jerk_trajectory(
    start_position: npt.ArrayLike, end_position: npt.ArrayLike, num_steps: int
) -> t.Iterator[npt.NDArray]:
    """Generator for computing minimum jerk trajectories.

    Example:
        To move from ``[0, 0, 0]`` to ``[1, 2, 3]`` over 1000 time steps:

        .. code-block:: Python

            for pos in min_jerk_trajectory([0, 0, 0], [1, 2, 3], 1000):
                robot.append_desired_action(Action(position=pos)

    Args:
        start_position: Joint positions where the trajectory starts.
        end_position: Joint positions where the trajectory ends.
        num_steps: Number of steps for the trajectory.
    """
    if num_steps < 1:
        raise ValueError("num_steps must be >= 1")

    start_position = np.asarray(start_position)
    end_position = np.asarray(end_position)
    position_delta = end_position - start_position

    for i in range(num_steps):
        alpha = i / num_steps
        step_position = start_position + position_delta * (
            10.0 * alpha**3 - 15.0 * alpha**4 + 6.0 * alpha**5
        )
        yield step_position
