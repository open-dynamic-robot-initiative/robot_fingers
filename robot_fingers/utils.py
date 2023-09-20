import time


class TimePrinter:
    """
    Regularly print the current date/time and the passed duration in hours.
    """

    def __init__(self, interval_s: float = 3600.0):
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

    def update(self):
        """Print time if the interval_s has passed since the last call."""
        now = time.time()
        if now - self._last_time_print > self.interval_s:
            time_str = time.strftime("%F %H:%M")
            duration_h = round((now - self.start_time) / 3600)
            print(
                "{} ({} h since {})".format(time_str, duration_h, self.start_time_str)
            )
            self._last_time_print = now
