#!/usr/bin/env python3
"""Parse post_submission logs to create object tracking quality plots.

The script trifingerpro_post_submission.py can write results of the tests to a
log file. This script reads that log to plot the results of the object
detection test over time.
"""
import argparse
import json
import os
import pathlib
import sys

import numpy as np
import matplotlib


try:
    import plotext

    has_plotext = True
except ImportError:
    has_plotext = False


def read_file(filename):
    alldata = []
    with open(filename, "r") as f:
        for line in f:
            if "Start post_submission" in line:
                # new sample
                data = {"position": None, "orientation": None}
            elif "INFO object_detection confidence" in line:
                json_str = line.rsplit(">>>", 1)[1]
                data["confidence"] = json.loads(json_str)
            elif "INFO object_detection position" in line:
                json_str = line.rsplit(">>>", 1)[1]
                data["position"] = json.loads(json_str)
            elif "INFO object_detection orientation" in line:
                json_str = line.rsplit(">>>", 1)[1]
                data["orientation"] = json.loads(json_str)
                alldata.append(data)
                data = {"position": None, "orientation": None}

    return alldata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logfiles", nargs="+", type=pathlib.Path, help="logfiles")
    parser.add_argument(
        "--no-display",
        dest="use_display",
        action="store_false",
        help="Act as if no display is available (mostly for debugging).",
    )
    args = parser.parse_args()

    has_display = args.use_display and "DISPLAY" in os.environ
    if not has_display:
        # Use a backend that doesn't need display
        print("No display found.  Set matplotlib backend to Agg.")
        matplotlib.use("Agg")

    # pyplot must be imported *after* setting the backend
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(4, 1)
    labels = [x.name for x in args.logfiles]
    for filename, label in zip(args.logfiles, labels):
        data = read_file(filename)

        position_displacement = [
            np.linalg.norm(s["position"]["mean"][:2]) for s in data
        ]
        mean_confidence = [s["confidence"]["object_mean_confidence"] for s in data]
        position_mean_error = [s["position"]["mean_error"] for s in data]
        orientation_mean_error = [s["orientation"]["mean_error"] for s in data]

        plot_data = (
            ("Mean Confidence", mean_confidence),
            ("Position Mean Error", position_mean_error),
            ("Orientation Mean Error", orientation_mean_error),
            ("Position Displacement from Center", position_displacement),
        )
        for ax, (title, data) in zip(axes, plot_data):
            ax.set_title(title)
            ax.plot(data, label=label)

    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper right")

    # to prevent overlap of subplots (must be called after creating the plots)
    fig.tight_layout()

    # First try to show plot directly with matplotlib.  If no display is
    # available, fall back to showing the plot in the terminal using plotext
    # (if available).  As last resort save the plot as image to /tmp
    if has_display:
        plt.show()
    elif has_plotext:
        plotext.from_matplotlib(fig)
        plotext.show()
    else:
        out_file = "/tmp/plot_post_submission_log.png"
        plt.savefig(out_file)
        print("Plot saved to file %s" % out_file)

    return 0


if __name__ == "__main__":
    sys.exit(main())
