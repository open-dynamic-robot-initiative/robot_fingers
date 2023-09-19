#!/usr/bin/env python3
"""Convert a binary log file into a plain text csv file."""
import argparse
import numpy as np
import progressbar

import robot_interfaces


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("infile", type=str, help="Binary input file.")
    parser.add_argument("outfile", type=str, help="Output CSV file.")
    parser.add_argument(
        "--number-of-fingers",
        "-n",
        type=int,
        choices=[1, 3],
        default=3,
        help="Number of fingers of the robot.",
    )
    args = parser.parse_args()

    if args.number_of_fingers == 1:
        module = robot_interfaces.finger
    else:
        module = robot_interfaces.trifinger

    log = module.BinaryLogReader(args.infile)

    n_fingers = args.number_of_fingers
    n_joints = n_fingers * 3

    header = ["#time_index", "timestamp"]
    header += ["status_action_repetitions", "status_error_status"]

    header += ["observation_position_%d" % i for i in range(n_joints)]
    header += ["observation_velocity_%d" % i for i in range(n_joints)]
    header += ["observation_torque_%d" % i for i in range(n_joints)]
    header += ["observation_tip_force_%d" % i for i in range(n_fingers)]

    for action_type in ("applied_action", "desired_action"):
        for field in ("torque", "position", "position_kp", "position_kd"):
            header += ["%s_%s_%d" % (action_type, field, i) for i in range(n_joints)]

    print("Convert log file")
    progress = progressbar.ProgressBar()
    all_data = []
    for entry in progress(log.data):
        data = [
            entry.timeindex,
            entry.timestamp,
            entry.status.action_repetitions,
            int(entry.status.error_status),
        ]

        data += [entry.observation.position[i] for i in range(n_joints)]
        data += [entry.observation.velocity[i] for i in range(n_joints)]
        data += [entry.observation.torque[i] for i in range(n_joints)]
        data += [entry.observation.tip_force[i] for i in range(n_fingers)]

        data += [entry.applied_action.torque[i] for i in range(n_joints)]
        data += [entry.applied_action.position[i] for i in range(n_joints)]
        data += [entry.applied_action.position_kp[i] for i in range(n_joints)]
        data += [entry.applied_action.position_kd[i] for i in range(n_joints)]

        data += [entry.desired_action.torque[i] for i in range(n_joints)]
        data += [entry.desired_action.position[i] for i in range(n_joints)]
        data += [entry.desired_action.position_kp[i] for i in range(n_joints)]
        data += [entry.desired_action.position_kd[i] for i in range(n_joints)]

        all_data.append(data)

    print("Write to file {}...".format(args.outfile))
    np.savetxt(args.outfile, all_data, header=" ".join(header), comments="")


if __name__ == "__main__":
    main()
