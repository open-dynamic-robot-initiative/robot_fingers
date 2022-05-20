#!/usr/bin/env python3
"""Construct a "swipe" trajectory to bring the object back to the centre."""
import argparse
import time
import progressbar

import numpy as np
from scipy.spatial.transform import Rotation

import trifinger_simulation


def min_jerk_trajectory(current, setpoint, frequency, avg_speed):
    """Compute minimum jerk trajectory.

    Based on [Mika's tech blog](https://mika-s.github.io/python/control-theory/trajectory-generation/2017/12/06/trajectory-generation-with-a-minimum-jerk-trajectory.html)  # noqa
    License: CC BY-SA 3.0
    """
    trajectory = []
    trajectory_derivative = []

    num_steps = np.linalg.norm(current - setpoint) / avg_speed
    move_time = num_steps / frequency

    timefreq = int(move_time * frequency)

    for t in range(1, timefreq):
        trajectory.append(
            current
            + (setpoint - current)
            * (
                10.0 * (t / timefreq) ** 3
                - 15.0 * (t / timefreq) ** 4
                + 6.0 * (t / timefreq) ** 5
            )
        )

        trajectory_derivative.append(
            frequency
            * (1.0 / timefreq)
            * (setpoint - current)
            * (
                30.0 * (t / timefreq) ** 2.0
                - 60.0 * (t / timefreq) ** 3.0
                + 30.0 * (t / timefreq) ** 4.0
            )
        )

    return trajectory, trajectory_derivative


def compute_single_swipe_trajectory(
    start_xy,
    end_xy,
    quicktravel_height,
    swipe_height,
    max_speed_mps,
    rate_hz=1000,
):
    """Compute end-effector-trajectory of a single swipe.

    The trajectory starts at start_xy/quicktravel_height, then goes down to
    swipe_height, moves in a straight line to end_xy and there moves up to
    quicktravel_height again::

         Start                                 End
           |                                    ^       --- quicktravel_height
           |                                    |
           +------------------------------------+       --- swipe_height
        start_xy                              end_xy


    Args:
        start_xy: x,y-position of start point.
        end_xy: x,y-position of end-point.
        quicktravel_height: z-value of the "quick travel" height.  Here the
            end-effector is supposed to be able to move fast without hitting
            the object.
        swipe_height: z-value of the "swipe" height.  This should be low
            enough that the end-effector can move the object.
        max_speed_mps: Maximum speed [m/s] of the end-effector while moving on
            swipe height.
        rate_hz: Control rate of the robot [Hz].

    Returns:
        List of end-effector positions forming a "swipe trajectory" as
        described above.
    """
    # divide max. speed by 1.87 to get average speed
    avg_speed_mps = max_speed_mps / 1.87
    avg_speed_m_per_step = avg_speed_mps / rate_hz

    pos_start_high = np.array([start_xy[0], start_xy[1], quicktravel_height])
    pos_start_low = np.array([start_xy[0], start_xy[1], swipe_height])
    pos_end_low = np.array([end_xy[0], end_xy[1], swipe_height])
    pos_end_high = np.array([end_xy[0], end_xy[1], quicktravel_height])

    # go down sequence
    down_traj, _ = min_jerk_trajectory(
        pos_start_high, pos_start_low, rate_hz, avg_speed_m_per_step
    )

    # swipe sequence
    swipe_traj, _ = min_jerk_trajectory(
        pos_start_low, pos_end_low, rate_hz, avg_speed_m_per_step
    )

    # go up sequence
    up_traj, _ = min_jerk_trajectory(
        pos_end_low, pos_end_high, rate_hz, avg_speed_m_per_step
    )

    full_trajectory = np.vstack([down_traj, swipe_traj, up_traj])

    return full_trajectory


def connect_swipes(initial_position, swipes, jump_speed_mps, rate_hz=1000):
    """Connect the swipe trajectories with fast "jump" movements.

    Args:
        initial_position:  Initial end-effector position.
        swipes:  List of swipe trajectories.
        jump_speed_mps:  Average speed [m/s] with which the end-effector can
            travel from the end of one swipe trajectory to the start of the
            next one.
        rate_hz:  Control rate of the robot.

    Returns:
        Combined end-effector trajectory starting at initial_position and
        containing all the swipes, connected by fast "jump trajectories".
    """
    # divide max. speed by 1.87 to get average speed
    jump_speed_m_per_step = jump_speed_mps / rate_hz

    start = initial_position
    connected_swipes = []

    for swipe in swipes:
        jump_traj, _ = min_jerk_trajectory(
            start, swipe[0], rate_hz, jump_speed_m_per_step
        )
        connected_swipes.append(jump_traj)
        connected_swipes.append(swipe)

        start = swipe[-1]

    return np.vstack(connected_swipes)


def trajectory_ik(traj_0, traj_120, traj_240, visualize):
    """Convert list of end-effector positions into list of joint positions.

    Uses trifinger_simulation to convert the end-effector trajectories of the
    three finger tips into joint positions using inverse kinematics.

    Args:
        traj_0:  End-effector trajectory of finger 0.
        traj_120:  End-effector trajectory of finger 120.
        traj_240:  End-effector trajectory of finger 240.
        visualize:  If True, show simulation visualization.

    Returns:
        Trajectory in joint space.
    """
    time_step = 0.001
    finger = trifinger_simulation.SimFinger(
        finger_type="trifingerpro",
        time_step=time_step,
        enable_visualization=visualize,
    )
    init_pos = np.array([0, 0.9, -1.7] * finger.number_of_fingers)
    finger.reset_finger_positions_and_velocities(init_pos)

    # get initial observation
    finger_action = finger.Action(position=init_pos)
    t = finger.append_desired_action(finger_action)
    obs = finger.get_observation(t)

    joint_trajectory = []
    progress = progressbar.ProgressBar(maxval=len(traj_0))
    progress.start()
    i = 0
    for tip_positions in zip(traj_0, traj_120, traj_240):
        progress.update(i)
        i += 1

        joint_pos, err = finger.kinematics.inverse_kinematics(
            tip_positions, obs.position
        )
        joint_trajectory.append(joint_pos)

        finger_action = finger.Action(position=joint_pos)
        t = finger.append_desired_action(finger_action)
        obs = finger.get_observation(t)
        time.sleep(time_step)

    progress.finish()

    return joint_trajectory


def compute_all_finger_multi_swipe(
    start_xy,
    end_xy,
    quicktravel_height,
    swipe_height,
    max_speed_mps,
    angle_range,
):
    """Compute a "swipe towards centre" trajectory using all three fingers.

    The trajectory consists of multiple swipes which are performed on all
    fingers symmetrically.  A "canonical swipe" is computed for finger 0 using
    start_xy and end_xy.  This is then duplicated and rotated according to the
    steps in angle_range.
    Symmetrically equivalent swipes are computed for finger 120 and finger 240.
    """
    swipe = compute_single_swipe_trajectory(
        start_xy, end_xy, quicktravel_height, swipe_height, max_speed_mps
    )

    rot_finger120 = Rotation.from_euler("z", -120, degrees=True)
    rot_finger240 = Rotation.from_euler("z", -240, degrees=True)

    swipes_0 = []
    swipes_120 = []
    swipes_240 = []
    for angle in angle_range:
        rot = Rotation.from_euler("z", angle, degrees=True)
        rotated_swipe_0 = rot.apply(swipe)
        rotated_swipe_120 = rot_finger120.apply(rotated_swipe_0)
        rotated_swipe_240 = rot_finger240.apply(rotated_swipe_0)

        swipes_0.append(rotated_swipe_0)
        swipes_120.append(rotated_swipe_120)
        swipes_240.append(rotated_swipe_240)

    return (swipes_0, swipes_120, swipes_240)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("out_file", type=str)
    parser.add_argument("--visualize", "-v", action="store_true")
    args = parser.parse_args()

    quicktravel_height = 0.06
    swipe_height = 0.025

    # compute a outer swipe trajectory
    start_xy = np.array([0.0, 0.2])
    end_xy = np.array([0.0, 0.05])
    max_speed_mps = 0.2
    angle_range = range(-70, 61, 10)

    outer_swipes = compute_all_finger_multi_swipe(
        start_xy,
        end_xy,
        quicktravel_height,
        swipe_height,
        max_speed_mps,
        angle_range,
    )

    # compute a inner swipe trajectory
    start_xy = np.array([0.0, 0.12])
    end_xy = np.array([0.0, 0.06])
    max_speed_mps = 0.1
    angle_range = range(60, -61, -15)

    inner_swipes = compute_all_finger_multi_swipe(
        start_xy,
        end_xy,
        quicktravel_height,
        swipe_height,
        max_speed_mps,
        angle_range,
    )

    swipes_0 = outer_swipes[0] + inner_swipes[0]
    swipes_120 = outer_swipes[1] + inner_swipes[1]
    swipes_240 = outer_swipes[2] + inner_swipes[2]

    initial_pos_0 = [0.08457, 0.059190205160135, 0.07725789413684458]
    initial_pos_120 = [
        0.008975221324218269,
        -0.10283487097808879,
        0.07725789413684458,
    ]
    initial_pos_240 = [
        -0.09354522132373216,
        0.043644665818320084,
        0.07725789413684458,
    ]

    jump_speed_mps = 0.5
    swipes_0 = connect_swipes(initial_pos_0, swipes_0, jump_speed_mps)
    swipes_120 = connect_swipes(initial_pos_120, swipes_120, jump_speed_mps)
    swipes_240 = connect_swipes(initial_pos_240, swipes_240, jump_speed_mps)

    joint_trajectory = trajectory_ik(
        swipes_0, swipes_120, swipes_240, args.visualize
    )

    header = ["observation_position_%d" % i for i in range(9)]
    np.savetxt(
        args.out_file, joint_trajectory, header=" ".join(header), comments=""
    )


if __name__ == "__main__":
    main()
