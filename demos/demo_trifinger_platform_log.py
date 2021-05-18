#!/usr/bin/env python3
"""Demo showing how to use TriFingerPlatformLog."""
import argparse

import cv2

import robot_fingers
from trifinger_cameras import utils


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("robot_log", type=str, help="Robot log file")
    parser.add_argument("camera_log", type=str, help="Camera log file")
    args = parser.parse_args()

    log = robot_fingers.TriFingerPlatformLog(args.robot_log, args.camera_log)

    # iterate over all robot time steps in the log
    for t in range(log.get_first_timeindex(), log.get_last_timeindex() + 1):
        # TriFingerPlatformLog provides the same getters as
        # TriFingerPlatformFrontend:

        robot_observation = log.get_robot_observation(t)
        # print time index and position of the first finger
        print("%d - %s" % (t, robot_observation.position[:3]))

        # show images of camera180
        try:
            camera_observation = log.get_camera_observation(t)
            cv2.imshow(
                "camera180",
                utils.convert_image(camera_observation.cameras[1].image),
            )
            key = cv2.waitKey(1)

            if key == ord("q"):
                return
        except Exception as e:
            print(e)


if __name__ == "__main__":
    main()
