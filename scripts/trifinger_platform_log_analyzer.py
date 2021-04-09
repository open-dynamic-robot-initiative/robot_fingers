#!/usr/bin/env python3
"""Run some analysis on a TriFinger platform log (i.e. robot + camera log)."""
import argparse

import matplotlib.pyplot as plt

import robot_fingers


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("robot_log", type=str, help="Robot log file")
    parser.add_argument("camera_log", type=str, help="Camera log file")
    args = parser.parse_args()

    log = robot_fingers.TriFingerPlatformLog(args.robot_log, args.camera_log)

    robot_timestamps = []
    camera_timestamps = []

    # iterate over all robot time steps in the log
    for t in range(log.get_first_timeindex(), log.get_last_timeindex() + 1):
        robot_timestamps.append(log.get_timestamp_ms(t))

        # show images of camera180
        try:
            camera_observation = log.get_camera_observation(t)
            camera_timestamps.append(camera_observation.cameras[0].timestamp)
        except Exception as e:
            print(e)
            camera_timestamps.append(0)

    plt.plot(robot_timestamps, label="robot")
    plt.plot(camera_timestamps, label="camera")
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()
