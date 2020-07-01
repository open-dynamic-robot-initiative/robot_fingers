"""Functions for sampling, validating and evaluating "move cube" goals."""
import random

import numpy as np
from scipy.spatial.transform import Rotation


# Number of time steps in one episode
episode_length = 5000  # TODO set actual value


_CUBE_WIDTH = 0.065
_ARENA_RADIUS = 0.195

_cube_3d_radius = _CUBE_WIDTH * np.sqrt(3) / 2
_max_cube_com_distance_to_center = _ARENA_RADIUS - _cube_3d_radius

_min_height = _CUBE_WIDTH / 2
_max_height = 0.1  # TODO


_cube_corners = np.array(
    [
        [-1, -1, -1],
        [-1, -1, +1],
        [-1, +1, -1],
        [-1, +1, +1],
        [+1, -1, -1],
        [+1, -1, +1],
        [+1, +1, -1],
        [+1, +1, +1],
    ]
) * (_CUBE_WIDTH / 2)


class InvalidGoalError(Exception):
    """Exception used to indicate that the given goal is invalid."""

    def __init__(self, message, position, orientation):
        super().__init__(message)
        self.position = position
        self.orientation = orientation


def get_cube_corner_positions(position, orientation):
    """Get the positions of the cube's corners with the given pose.

    Args:
        position: Position (x, y, z) of the cube.
        orientation: Quaternion (x, y, z, w) with the orientation of the cube.

    Returns:
        (array, shape=(8, 3)): Positions of the corners of the cube in the
            given pose.
    """
    rotation = Rotation.from_quat(orientation)
    translation = np.asarray(position)

    return rotation.apply(_cube_corners) + translation


def sample_goal(difficulty, current_position=None, current_orientation=None):
    """Sample a goal pose for the cube.

    Args:
        difficulty (int):  Difficulty level.  The higher, the more difficult is
            the goal.  Possible levels are:

            - 1: Goal is on the ground with a random rotation around the
                  z-axis.
            - 2: The goal can be above the ground.  Rotation is limited to the
                  z-axis.

        current_position:  Current (x, y, z)-position of the cube.  Currently
            unused.
        current_orientation:  Current orientation of the cube as quaternion
            (x, y, z, w).  If set, it is considered while sampling the goal
            orientation.  If not set, it is assumed that the cube is currently
            aligned with the axes of the world frame.  This is relevant for
            some difficulty levels to ensure that the goal orientation only
            differs from the current one by what is specified for that
            difficulty level (e.g. only rotation around z-axes for level 1).

    Returns:
        (tuple): A tuple with the goal position (x, y, z) and the goal
        orientation quaternion (x, y, z, w) relative to the world frame.
    """
    # difficulty -1 is for initialization

    # TODO Should we add a minimum distance to current_position to avoid goals
    # that are too easy?

    # sample uniform position in circle (https://stackoverflow.com/a/50746409)
    radius = _max_cube_com_distance_to_center * np.sqrt(random.random())
    theta = random.uniform(0, 2 * np.pi)

    # x,y-position of the cube
    x = radius * np.cos(theta)
    y = radius * np.sin(theta)

    if difficulty == -1 or difficulty == 1:
        # on the ground, random yaw
        z = _CUBE_WIDTH / 2
    elif difficulty == 2:
        # in the air, random yaw
        z = random.uniform(_min_height, _max_height)
    else:
        raise ValueError("Invalid difficulty %d" % difficulty)

    position = np.array((x, y, z))

    # random yaw angle (relative to current_orientation if given)
    yaw = random.uniform(0, 2 * np.pi)

    orientation = Rotation.from_euler("z", yaw)
    if current_orientation is not None:
        orientation = orientation * Rotation.from_quat(current_orientation)

    return position, orientation.as_quat()


def validate_goal(position, orientation):
    """Validate that the given pose is a valid goal (e.g. no collision)

    Raises an error if the given goal pose is invalid.

    Args:
        position:  Goal position.
        orientation:  Goal orientation.

    Raises:
        ValueError:  If given values are not a valid 3d position/orientation.
        InvalidGoalError:  If the given pose exceeds the allowed goal space.
    """
    if len(position) != 3:
        raise ValueError("len(position) != 3")
    if len(orientation) != 4:
        raise ValueError("len(orientation) != 4")
    if np.linalg.norm(position[:2]) > _max_cube_com_distance_to_center:
        raise InvalidGoalError(
            "Position is outside of the arena circle.", position, orientation
        )
    if position[2] < _min_height:
        raise InvalidGoalError("Position is too low.", position, orientation)
    if position[2] > _max_height:
        raise InvalidGoalError("Position is too high.", position, orientation)

    # even if the CoM is above _min_height, a corner could be intersecting with
    # the bottom depending on the orientation
    corners = get_cube_corner_positions(position, orientation)
    min_z = min(z for x, y, z in corners)
    # allow a bit below zero to compensate numerical inaccuracies
    if min_z < -1e-10:
        raise InvalidGoalError(
            "Position of a corner is too low (z = {}).".format(min_z),
            position,
            orientation,
        )


def evaluate_state(
    goal_position, goal_orientation, actual_position, actual_orientation
):
    """Compute cost of a given cube pose.  Less is better.

    Args:
        goal_position:  Goal position (x, y, z).
        goal_orientation:  Goal orientation as quaternion (x, y, z, q).
        actual_position:  Goal position (x, y, z).
        actual_orientation:  Goal orientation as quaternion (x, y, z, q).

    Returns:
        Cost of the actual pose w.r.t. to the goal pose.  Lower value means
        that the actual pose is closer to the goal.  Zero if actual == goal.
    """
    # Use DISP distance (max. displacement of the corners)
    goal_corners = get_cube_corner_positions(goal_position, goal_orientation)
    actual_corners = get_cube_corner_positions(
        actual_position, actual_orientation
    )

    disp = max(np.linalg.norm(goal_corners - actual_corners, axis=1))
    return disp
