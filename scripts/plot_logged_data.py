#!/usr/bin/env python3
"""Simply script to quickly plot data from a log file."""

import argparse
import numpy as np
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("filename", type=str)
    parser.add_argument("column_index", type=int, nargs="+")
    parser.add_argument("--base_index", type=int, default=0)
    args = parser.parse_args()

    data = np.loadtxt(args.filename)

    for i in args.column_index:
        plt.plot(data[:, args.base_index], data[:, i], label=i)

    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()
