#!/usr/bin/env python3
"""Demo showing how to use the Object Tracker Interface in simulation."""
import argparse
import numpy as np
import time

import trifinger_object_tracking.py_object_tracker as object_tracker
from trifinger_simulation import sim_finger, collision_objects, sample


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--save-poses",
        type=str,
        metavar="FILENAME",
        help="""If set, the demo stops when the history of the object pose time
            series is full and stores the buffered poses to the specified file.
        """,
    )
    parser.add_argument(
        "--history-size",
        type=int,
        default=1000,
        help="""Size of the object pose time series.  Default: %(default)d""",
    )
    parser.add_argument(
        "--non-real-time",
        action="store_true",
        help="""Do not run the simulation in real-time but as fast as
            possible.
        """,
    )
    args = parser.parse_args()

    real_time = not args.non_real_time
    time_step = 0.004
    finger = sim_finger.SimFinger(
        finger_type="trifingerone",
        time_step=time_step,
        enable_visualization=True,
    )

    # Object and Object Tracker Interface
    # ===================================
    #
    # Important:  These objects need to be created _after_ the simulation is
    # initialized (i.e. after the SimFinger instance is created).

    # spawn a cube in the arena
    cube = collision_objects.Block()

    # initialize the object tracker interface
    object_tracker_data = object_tracker.Data(
        "object_tracker", True, args.history_size
    )
    object_tracker_backend = object_tracker.SimulationBackend(
        object_tracker_data, cube, real_time
    )
    object_tracker_frontend = object_tracker.Frontend(object_tracker_data)

    # Move the fingers to random positions so that the cube is kicked around
    # (and thus it's position changes).
    while True:
        goal = np.array(
            sample.random_joint_positions(
                number_of_fingers=3,
                lower_bounds=[-1, -1, -2],
                upper_bounds=[1, 1, 2],
            )
        )
        finger_action = finger.Action(position=goal)

        for _ in range(50):
            t = finger.append_desired_action(finger_action)
            finger.get_observation(t)
            if real_time:
                time.sleep(time_step)

        # get the latest pose estimate of the cube and print it
        t_obj = object_tracker_frontend.get_current_timeindex()
        cube_pose = object_tracker_frontend.get_pose(t_obj)
        print("Cube Pose: %s" % cube_pose)

        # if --save-poses is set, stop when the buffer is full and write it to
        # a file
        if args.save_poses and t_obj > args.history_size:
            # the backend needs to be stopped before saving the buffered poses
            object_tracker_backend.stop()
            object_tracker_backend.store_buffered_data(args.save_poses)
            break


if __name__ == "__main__":
    main()
