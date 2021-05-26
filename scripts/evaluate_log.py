#!/usr/bin/env python3
"""
Compute accumulated reward for a given log file.
"""
import argparse
import json
import pathlib
import sys

import robot_fingers
from trifinger_cameras.utils import convert_image
from trifinger_object_tracking.py_lightblue_segmenter import segment_image
from trifinger_simulation.camera import load_camera_parameters


def compute_reward_move_cube(task, log, t, goal):
    camera_observation = log.get_camera_observation(t)
    cube_pose = camera_observation.object_pose
    reward = -task.evaluate_state(
        goal["goal"], cube_pose, int(goal["difficulty"])
    )
    return reward


def compute_reward_move_cube_on_trajectory(task, log, t, goal):
    camera_observation = log.get_camera_observation(t)
    cube_pose = camera_observation.object_pose
    reward = -task.evaluate_state(goal["goal"], t, cube_pose.position)
    return reward


def compute_reward_rearrange_dice(task, log, t, goal, goal_masks):
    camera_observation = log.get_camera_observation(t)
    masks = tuple(
        segment_image(convert_image(c.image))
        for c in camera_observation.cameras
    )
    reward = -task.evaluate_state(goal_masks, masks)
    return reward


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--task",
        required=True,
        choices=["move_cube", "move_cube_on_trajectory", "rearrange_dice"],
        help="Name of the task.",
    )
    parser.add_argument(
        "--log-dir",
        required=True,
        type=pathlib.Path,
        help="Directory containing the log files.",
    )
    args = parser.parse_args()

    # some setup depending on the selected task
    if args.task == "move_cube":
        import trifinger_simulation.tasks.move_cube as task

        compute_reward = compute_reward_move_cube
    elif args.task == "move_cube_on_trajectory":
        import trifinger_simulation.tasks.move_cube_on_trajectory as task

        compute_reward = compute_reward_move_cube_on_trajectory
    elif args.task == "rearrange_dice":
        import trifinger_simulation.tasks.rearrange_dice as task

        compute_reward = compute_reward_rearrange_dice
    else:
        raise ValueError(f"Invalid task {args.task}")

    # dictionary with additional, task-specific data that is passed to the
    # compute_reward function
    additional_data = {}

    # load goal from json file
    try:
        with open(args.log_dir / "goal.json", "r") as fh:
            goal = json.load(fh)
        task.validate_goal(goal["goal"])
    except Exception as e:
        print("Invalid goal file:", e)
        sys.exit(1)

    if args.task == "rearrange_dice":
        # for the rearrange_dice task, we need to generate the goal mask
        camera_params = load_camera_parameters(args.log_dir, "camera{id}.yml")
        additional_data["goal_masks"] = task.generate_goal_mask(
            camera_params, goal["goal"]
        )

    try:
        robot_log = str(args.log_dir / "robot_data.dat")
        camera_log = str(args.log_dir / "camera_data.dat")

        if args.task in ("move_cube", "move_cube_on_trajectory"):
            log = robot_fingers.TriFingerPlatformWithObjectLog(
                robot_log, camera_log
            )
        else:
            log = robot_fingers.TriFingerPlatformLog(robot_log, camera_log)
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

    if t_last != task.EPISODE_LENGTH - 1:
        print(
            "Invalid log: Last time index in robot log is {}. Expected {}.".format(
                t_last, task.EPISODE_LENGTH - 1
            )
        )
        sys.exit(1)

    cumulative_reward = 0
    for t in range(log.get_first_timeindex(), log.get_last_timeindex() + 1):
        reward = compute_reward(task, log, t, goal, **additional_data)
        cumulative_reward += reward

    print("Cumulative Reward:", cumulative_reward)


if __name__ == "__main__":
    main()
