#!/usr/bin/env python3
"""Parse post_submission logs to create object tracking quality plots.

The script trifingerpro_post_submission.py can write results of the tests to a
log file. This script reads that log to plot the results of the object
detection test over time.
"""
import argparse
import pathlib
import sys
import json

import numpy as np
import matplotlib.pyplot as plt


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
    parser.add_argument(
        "logfiles", nargs="+", type=pathlib.Path, help="logfiles"
    )
    args = parser.parse_args()

    data = []
    for filename in args.logfiles:
        data.extend(read_file(filename))

    position_displacement = [
        np.linalg.norm(s["position"]["mean"][:2]) for s in data
    ]
    mean_confidence = [s["confidence"]["object_mean_confidence"] for s in data]
    position_mean_error = [s["position"]["mean_error"] for s in data]
    orientation_mean_error = [s["orientation"]["mean_error"] for s in data]

    fig, axes = plt.subplots(4, 1)
    plot_data = (
        ("Mean Confidence", mean_confidence),
        ("Position Mean Error", position_mean_error),
        ("Orientation Mean Error", orientation_mean_error),
        ("Position Displacement from Center", position_displacement),
    )
    for ax, (title, data) in zip(axes, plot_data):
        ax.set_title(title)
        ax.plot(data)

    # to prevent overlap of subplots (must be called after creating the plots)
    fig.tight_layout()

    plt.show()

    return 0


if __name__ == "__main__":
    sys.exit(main())
