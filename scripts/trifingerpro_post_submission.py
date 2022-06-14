#!/usr/bin/env python3
"""Perform "post submission" actions on TriFingerPro.

Performs the following actins:

- run some self-tests to check if the robot is behaving as expected
- move the fingers on a previously recorded trajectory to randomize the
  position of the cube
- use the cameras to check if the cube is still inside the arena (not
  implemented yet)
"""
import argparse
import json
import os
import sys
import typing
import logging
import logging.handlers

import numpy as np
import pandas
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory
import tomli

import robot_interfaces
import robot_fingers
import trifinger_object_tracking.py_tricamera_types as tricamera
import trifinger_object_tracking.py_object_tracker as object_tracker


# Distance from the zero position (finger pointing straight down) to the
# end-stop.
_zero_to_endstop = np.array([2.136, 2.442, -2.710] * 3)

# Maximum torque with signs same as for end-stop search
_max_torque_against_homing_endstop = [+0.4, +0.4, -0.4] * 3

# Tolerance when checking if expected position is reached
_position_tolerance = 0.1

_submission_system_config_file = "/etc/trifingerpro/submission_system.toml"


class NumpyEncoder(json.JSONEncoder):
    # From https://stackoverflow.com/a/47626762 (CC BY-SA 4.0)
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)


class StructuredMessage:
    """Log message with keywords for structured logging.

    Taken from
    https://docs.python.org/3.8/howto/logging-cookbook.html#implementing-structured-logging
    """

    def __init__(self, message, /, **kwargs):
        self.message = message
        self.kwargs = kwargs

    def __str__(self):
        return "%s >>> %s" % (
            self.message,
            json.dumps(self.kwargs, cls=NumpyEncoder),
        )


# for shorter code below
SM = StructuredMessage


def orientation_distance(rot1: Rotation, rot2: Rotation) -> float:
    """Compute angular distance between to orientations."""
    error_rot = rot2.inv() * rot1
    return error_rot.magnitude()


def load_object_type() -> typing.Optional[str]:
    with open(_submission_system_config_file, "rb") as f:
        config = tomli.load(f)

    try:
        return config["post_submission"]["object_type"]
    except KeyError:
        print("ERROR: failed to load object from config file.")
        return None


def get_robot_config_without_position_limits() -> (
    robot_fingers.TriFingerConfig
):
    """Get TriFingerPro configuration without position limits.

    Loads the TriFingerPro configuration from the default config file and
    disables the position limits.
    """
    # get the path to the TriFingerPro config file
    _, _, config_file = robot_fingers.robot.robot_configs["trifingerpro"]
    config_file_path = robot_fingers.robot.get_config_dir() / config_file

    # load the config
    config = robot_fingers.TriFingerConfig.load_config(str(config_file_path))

    # disable position limits
    n_joints = 9
    config.hard_position_limits_lower = [-np.inf] * n_joints
    config.hard_position_limits_upper = [np.inf] * n_joints
    config.soft_position_limits_lower = [-np.inf] * n_joints
    config.soft_position_limits_upper = [np.inf] * n_joints

    return config


def end_stop_check(robot: robot_fingers.Robot, log: logging.Logger) -> None:
    """Move robot to endstop, using constant torque and verify its position.

    Applies a constant torque for a fixed time to move to the end-stop.  If
    everything is okay, the robot should be at the end-stop position after this
    time.  If it is not, trigger a fault.

    Then move back to the initial position in a controlled way and again verify
    that the goal was reached successfully.

    Args:
        robot:  Initialized robot instance of the TriFingerPro.
        log: Logger instance to log results.
    """
    # go to the "homing" end-stop (using the same torque as during homing)
    action = robot.Action(
        torque=robot.config.calibration.endstop_search_torques_Nm
    )
    for _ in range(2000):
        t = robot.frontend.append_desired_action(action)
        robot.frontend.wait_until_timeindex(t)

    # push with maximum torque for a moment for slipping joint detection
    action = robot.Action(torque=_max_torque_against_homing_endstop)
    for _ in range(1000):
        t = robot.frontend.append_desired_action(action)
        robot.frontend.wait_until_timeindex(t)

    # release motors, so the joints are not actively pushing against the
    # end-stop anymore
    action = robot.Action()
    for _ in range(500):
        t = robot.frontend.append_desired_action(action)
        robot.frontend.wait_until_timeindex(t)

    observation = robot.frontend.get_observation(t)

    if (
        np.linalg.norm(_zero_to_endstop - observation.position)
        > _position_tolerance
    ):
        log.error(
            SM(
                "End stop not at expected position.",
                expected_position=_zero_to_endstop,
                actual_position=observation.position,
            )
        )
        sys.exit(1)

    # move back joint-by-joint
    goals = [
        (1000, np.array([0, np.nan, np.nan] * 3)),
        (500, np.array([0, 1.1, np.nan] * 3)),
        (500, np.array([0, 1.1, -1.9] * 3)),
    ]
    for duration, goal in goals:
        action = robot.Action(position=goal)
        for _ in range(duration):
            t = robot.frontend.append_desired_action(action)
            robot.frontend.wait_until_timeindex(t)

    observation = robot.frontend.get_observation(t)

    # verify that goal is reached
    if np.linalg.norm(goal - observation.position) > _position_tolerance:
        log.error(
            SM(
                "Robot did not reach goal position",
                desired_position=goal,
                actual_position=observation.position,
            )
        )
        sys.exit(1)


def run_self_test(robot: robot_fingers.Robot, log: logging.Logger) -> None:
    position_tolerance = 0.2
    push_sensor_threshold = 0.5

    initial_pose = [0.0, 1.1, -1.9] * 3

    reachable_goals = [
        [0.9, 1.5, -2.6] * 3,
        [-0.3, 1.5, -1.7] * 3,
    ]

    unreachable_goals = [
        [0.0, 0.0, 0.0] * 3,
        [-0.5, 1.5, 0.0] * 3,
    ]

    for goal in reachable_goals:
        action = robot.Action(position=goal)
        for _ in range(1000):
            t = robot.frontend.append_desired_action(action)
            robot.frontend.wait_until_timeindex(t)

        observation = robot.frontend.get_observation(t)

        # verify that goal is reached
        if np.linalg.norm(goal - observation.position) > position_tolerance:
            log.error(
                SM(
                    "Robot did not reach goal position",
                    desired_position=goal,
                    actual_position=observation.position,
                )
            )
            sys.exit(1)

        if (observation.tip_force > push_sensor_threshold).any():
            log.error(
                SM(
                    "Push sensor reports high value in non-contact situation.",
                    sensor_value=observation.tip_force,
                    desired_position=goal,
                    actual_position=observation.position,
                )
            )
            # sys.exit(1)

    for goal in unreachable_goals:
        # move to initial position first
        action = robot.Action(position=initial_pose)
        for _ in range(1000):
            t = robot.frontend.append_desired_action(action)
            robot.frontend.wait_until_timeindex(t)

        action = robot.Action(position=goal)
        for _ in range(1000):
            t = robot.frontend.append_desired_action(action)
            robot.frontend.wait_until_timeindex(t)

        observation = robot.frontend.get_observation(t)

        # verify that goal is reached
        if np.linalg.norm(goal - observation.position) < position_tolerance:
            log.error(
                SM(
                    "Robot reached a goal which should not be reachable.",
                    desired_position=goal,
                    actual_position=observation.position,
                )
            )
            sys.exit(1)

        if (observation.tip_force < push_sensor_threshold).any():
            log.error(
                SM(
                    "Push sensor reports low value in contact situation.",
                    sensor_value=observation.tip_force,
                    desired_position=goal,
                    actual_position=observation.position,
                )
            )
            # sys.exit(1)

    print("Test successful.")


def reset_object(robot, trajectory_file):
    """Replay a recorded trajectory to reset/randomise the object pose.

    Args:
        robot: The robot object.
        trajectory_file: Path to a CSV file defining the trajectory.
    """
    trajectory_file = os.path.join(
        get_package_share_directory("robot_fingers"),
        "config",
        trajectory_file,
    )
    data = pandas.read_csv(
        trajectory_file, delim_whitespace=True, header=0, low_memory=False
    )

    # determine number of joints
    key_pattern = "observation_position_{}"
    data_keys = [key_pattern.format(i) for i in range(9)]

    # extract the positions from the recorded data
    positions = data[data_keys].to_numpy()

    for position in positions:
        action = robot.Action(position=position)
        t = robot.frontend.append_desired_action(action)
        robot.frontend.wait_until_timeindex(t)


def check_object_detection_noise(
    object_type: str, log: logging.Logger
) -> bool:
    """
    Compute the variance of the object pose while nothing is moving and check
    against some limits.

    Args:
        object_type: Which object to look for ("cube" or "cuboid").
        log: Logger instance to log results.

    Returns:
        True if test is successful, False if there is any issue.
    """
    object_models = {
        "cube": "cube_v2",
        "cuboid": "cuboid_2x2x8_v2",
    }
    object_withs = {
        "cube": 0.035,
        "cuboid": 0.01,
    }

    N_SAMPLES = 30
    CONFIDENCE_LIMIT = 0.75
    POSITION_VAR_LIMIT = 1.0  # TODO good value?
    Z_POSITION_TOLERANCE = 0.01
    ORIENTATION_DIFF_LIMIT = 1.0  # TODO good value?

    expected_z_pos = object_withs[object_type]

    camera_data = tricamera.SingleProcessData(history_size=N_SAMPLES)
    model = object_tracker.get_model_by_name(object_models[object_type])
    camera_driver = tricamera.TriCameraObjectTrackerDriver(
        "camera60", "camera180", "camera300", model
    )
    camera_backend = tricamera.Backend(camera_driver, camera_data)
    camera_frontend = tricamera.Frontend(camera_data)

    # collect observations
    observation_buffer: typing.List[object_tracker.ObjectPose] = []
    t = camera_frontend.get_current_timeindex()
    while len(observation_buffer) < N_SAMPLES:
        obs = camera_frontend.get_observation(t)
        observation_buffer.append(obs.object_pose)
        t += 1

    camera_backend.shutdown()

    mean_confidence = np.mean([p.confidence for p in observation_buffer])

    log.info(SM("confidence", object_mean_confidence=mean_confidence))

    if mean_confidence < CONFIDENCE_LIMIT:
        log.error(
            SM(
                "Object detection confidence is too low",
                mean_confidence=mean_confidence,
                limit=CONFIDENCE_LIMIT,
            )
        )
        return False

    # Only check position/orientation if confidence test has passed (if no
    # object is found, these will contain invalid values, causing errors)

    mean_position = np.mean([p.position for p in observation_buffer], axis=0)
    var_position = np.var([p.position for p in observation_buffer], axis=0)
    # for orientation use scipy Rotation to compute mean and then compute the
    # mean angular difference of each orientation to this mean.
    orientations = Rotation.from_quat(
        [p.orientation for p in observation_buffer]
    )
    mean_orientation = orientations.mean()
    orientations_diff_to_mean = [
        orientation_distance(o, mean_orientation) for o in orientations
    ]
    mean_orientation_diff = np.mean(orientations_diff_to_mean)

    log.info(SM("position mean", object_mean_position=mean_position))
    log.info(SM("position var", object_var_position=var_position))
    log.info(
        SM("orientation diff", mean_orientation_diff=mean_orientation_diff)
    )

    if np.max(var_position) > POSITION_VAR_LIMIT:
        log.error(
            SM(
                "Object position variance exceeds limit",
                variance=var_position,
                limit=POSITION_VAR_LIMIT,
            )
        )
        return False

    if abs(mean_position[2] - expected_z_pos) > Z_POSITION_TOLERANCE:
        log.error(
            SM(
                "Object height exceeds tolerance",
                z_pos=mean_position[2],
                expected_z_pos=expected_z_pos,
                tolerance=Z_POSITION_TOLERANCE,
            )
        )
        return False

    if mean_orientation_diff > ORIENTATION_DIFF_LIMIT:
        log.error(
            SM(
                "Object orientation variance exceeds limit",
                mean_orientation_diff=mean_orientation_diff,
                limit=ORIENTATION_DIFF_LIMIT,
            )
        )
        return False

    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--object",
        type=str,
        metavar="OBJECT_TYPE",
        choices=["cube", "cuboid", "dice", "auto"],
        help="""Specify with which object the robot is equipped (if any).  If
            set to "auto", the object type is read from the submission system
            configuration.
        """,
    )
    parser.add_argument(
        "--reset",
        action="store_true",
        help="""Execute a trajectory to reset the object.  Only valid if
            --object is set.
        """,
    )
    parser.add_argument(
        "--log",
        dest="logfile",
        metavar="LOGFILE",
        type=str,
        help="If set, safe log to the specified file (using file rotation).",
    )
    parser.add_argument(
        "--skip-robot-test",
        action="store_true",
        help="Skip the robot self-test.",
    )
    args = parser.parse_args()

    # configure logger
    log = logging.getLogger()
    log.setLevel(logging.NOTSET)
    stdout_handler = logging.StreamHandler(sys.stdout)
    stdout_handler.setFormatter(
        logging.Formatter("%(levelname)s - %(message)s")
    )
    stdout_handler.setLevel(logging.INFO)
    log.addHandler(stdout_handler)
    if args.logfile:
        file_handler = logging.handlers.RotatingFileHandler(
            filename=args.logfile, maxBytes=50000000, backupCount=5
        )
        file_handler.setFormatter(
            logging.Formatter(
                "%(asctime)s %(levelname)s %(name)s %(message)s",
                datefmt="%Y-%m-%dT%H:%M:%S%z",
            )
        )
        file_handler.setLevel(logging.DEBUG)
        log.addHandler(file_handler)

    # log parameters
    log.debug(SM("Start post_submission", **vars(args)))

    if args.object == "auto":
        args.object = load_object_type()

    robot = None
    if not args.skip_robot_test or args.reset:
        print("Initialise robot.")
        config = get_robot_config_without_position_limits()
        robot = robot_fingers.Robot(
            robot_interfaces.trifinger,
            robot_fingers.create_trifinger_backend,
            config,
        )
        robot.initialize()

    if not args.skip_robot_test:
        print("End stop test")
        end_stop_check(robot, logging.getLogger("end_stop_test"))
        print("Position reachability test")
        run_self_test(robot, logging.getLogger("self_test"))

    if args.reset:
        if args.object == "cube":
            print("Reset cube position")
            reset_object(
                robot, "trifingerpro_shuffle_cube_trajectory_fast.csv"
            )
        elif args.object == "cuboid":
            print("Reset cuboid position")
            reset_object(robot, "trifingerpro_recenter_cuboid_2x2x8.csv")
        elif args.object == "dice":
            print("Shuffle dice positions")
            reset_object(robot, "trifingerpro_shuffle_dice_trajectory.csv")

    # terminate the robot
    del robot

    if args.object in ["cube", "cuboid"]:
        print("Check object detection")
        if not check_object_detection_noise(
            args.object, logging.getLogger("object_detection")
        ):
            sys.exit(2)


if __name__ == "__main__":
    main()
