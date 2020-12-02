#!/usr/bin/env python3
"""
Compute accumulated reward for a given log file.
"""
import argparse
import json
import sys

import robot_fingers
from trifinger_simulation.tasks import move_cube


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--robot-log",
        required=True,
        type=str,
        help="Path to the robot log file.",
    )
    parser.add_argument(
        "--camera-log",
        required=True,
        type=str,
        help="Path to the camera log file.",
    )
    parser.add_argument(
        "--goal-file",
        required=True,
        type=str,
        help="Path to the goal JSON file.",
    )
    args = parser.parse_args()

    # load goal from json file
    move_cube.validate_goal_file(args.goal_file)
    with open(args.goal_file, "r") as fh:
        goal = json.load(fh)
    difficulty = int(goal["difficulty"])
    goal_pose = move_cube.Pose.from_dict(goal["goal"])

    try:
        log = robot_fingers.TriFingerPlatformLog(
            args.robot_log, args.camera_log
        )
    except Exception as e:
        print("Failed to load logs:", e)
        sys.exit(1)

    # verify that the log is complete and matches the expected length
    t_first = log.get_first_timeindex()
    t_last = log.get_last_timeindex()

    if t_first != 0:
        print(
            "Invalid log: First time index in robot log is {}. Expected 0.".format(
                t_first
            )
        )
        sys.exit(1)

    if t_last != move_cube.episode_length - 1:
        print(
            "Invalid log: Last time index in robot log is {}. Expected {}.".format(
                t_last, move_cube.episode_length - 1
            )
        )
        sys.exit(1)

    cumulative_reward = 0
    for t in range(log.get_first_timeindex(), log.get_last_timeindex() + 1):
        camera_observation = log.get_camera_observation(t)
        cube_pose = camera_observation.object_pose

        reward = -move_cube.evaluate_state(goal_pose, cube_pose, difficulty)
        cumulative_reward += reward

    print("Cumulative Reward:", cumulative_reward)


if __name__ == "__main__":
    main()
