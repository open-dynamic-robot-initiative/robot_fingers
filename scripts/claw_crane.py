#!/usr/bin/env python3
"""Move the TriFingerPro robot like a claw crane using the keyboard."""
import argparse
import curses
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory

import trifinger_simulation

import robot_fingers
import robot_properties_fingers
from robot_fingers.curses import SimpleCursesGUI


def loop(win, args):
    title = "Claw Crane"
    status_line = " q: quit | a/d: x-axis | w/s: y-axis | r/f: z-axis | q/e: open/close"

    robot_properties_path = get_package_share_directory("robot_properties_fingers")
    urdf_file = trifinger_simulation.finger_types_data.get_finger_urdf("trifingerpro")
    finger_urdf_path = os.path.join(robot_properties_path, "urdf", urdf_file)
    kinematics = robot_properties_fingers.Kinematics(
        finger_urdf_path,
        ["finger_tip_link_0", "finger_tip_link_120", "finger_tip_link_240"],
    )

    robot = robot_fingers.Robot.create_by_name("trifingerpro")
    robot.initialize()

    gui = SimpleCursesGUI(win, title, status_line)

    center_position = np.array([0, 0, 0.07])
    dist_tips_to_center = 0.07
    step_size = 0.002

    rot120 = Rotation.from_euler("z", -120, degrees=True)
    vec0 = np.array([0, 1.0, 0])
    vec120 = rot120.apply(vec0)
    vec240 = rot120.apply(vec120)

    current_joint_pos = np.array([0.39, 1.0, -1.77] * 3)
    target_joint_pos = current_joint_pos

    try:
        while True:
            steps = 200
            joint_pos_step_size = (target_joint_pos - current_joint_pos) / steps

            for _ in range(50):
                current_joint_pos += joint_pos_step_size
                finger_action = robot.Action(position=current_joint_pos)
                t = robot.frontend.append_desired_action(finger_action)
            obs = robot.frontend.get_observation(t)

            gui.update_screen(
                [
                    "Center: ({:.3f}, {:.3f}, {:.3f})".format(*center_position),
                    "Tips to Center: {:.3f}".format(dist_tips_to_center),
                    "",
                ]
            )

            key = gui.get_pressed_key()
            if key == 27:  # ESC
                # quit
                return
            elif key == ord("a"):
                center_position += [-step_size, 0, 0]
            elif key == ord("d"):
                center_position += [step_size, 0, 0]
            elif key == ord("w"):
                center_position += [0, step_size, 0]
            elif key == ord("s"):
                center_position += [0, -step_size, 0]
            elif key == ord("r"):
                center_position += [0, 0, step_size]
            elif key == ord("f"):
                center_position += [0, 0, -step_size]
            elif key == ord("q"):
                dist_tips_to_center += step_size
            elif key == ord("e"):
                dist_tips_to_center -= step_size
            else:
                # If there is no update, there is also no need to compute a new
                # action, so skip the rest of the loop.
                continue

            # tip of finger 0 is moving on the y-axis
            tip_pos_0 = center_position + vec0 * dist_tips_to_center

            # tips of fingers 120 and 240 are displaced from the center along
            # the vec120 and vec240 vectors
            tip_pos_120 = center_position + vec120 * dist_tips_to_center
            tip_pos_240 = center_position + vec240 * dist_tips_to_center

            goal = [tip_pos_0, tip_pos_120, tip_pos_240]
            target_joint_pos, err = kinematics.inverse_kinematics(
                goal, obs.position, tolerance=0.002, max_iterations=3000
            )

    except Exception as e:
        gui.display_error(str(e))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    args = parser.parse_args()

    curses.wrapper(lambda stdscr: loop(stdscr, args))

    return 0


if __name__ == "__main__":
    sys.exit(main())
