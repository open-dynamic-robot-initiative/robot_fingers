#!/usr/bin/env python3
"""Plot the data of a given run duration log file."""
import argparse
import datetime
import pathlib
import sys

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.dates


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "logfile", type=pathlib.Path, help="Path to the logfile."
    )
    args = parser.parse_args()

    data = np.loadtxt(args.logfile)

    timestamps = [datetime.datetime.fromtimestamp(t) for t in data[:, 0]]
    cumsum = data[:, 1].cumsum()

    # rescale from steps to hours (assuming 1 step == 1 ms)
    cumsum /= 3600000

    fig, ax = plt.subplots(1, 1)

    ax.fill_between(timestamps, cumsum)

    ax.set_title(args.logfile.stem)
    ax.set_xlabel("Date")
    ax.set_ylabel("Accumulated Robot Run Duration [h]")

    ax.xaxis.set_major_formatter(matplotlib.dates.DateFormatter("%Y-%m-%d"))

    ax.text(
        0.05,
        0.95,
        "total run duration: %.1f h" % cumsum[-1],
        transform=ax.transAxes,
        verticalalignment="top",
        bbox=dict(boxstyle="round", alpha=0.5),
    )

    plt.show()

    return 0


if __name__ == "__main__":
    sys.exit(main())
