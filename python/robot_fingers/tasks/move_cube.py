import random

import numpy as np
from scipy.spatial.transform import Rotation


CUBE_WIDTH = 0.065
ARENA_RADIUS = 0.195

cube_3d_radius = CUBE_WIDTH * np.sqrt(3) / 2
max_radius = ARENA_RADIUS - cube_3d_radius  # FIXME better name

min_height = CUBE_WIDTH / 2
max_height = 0.1  # TODO


cube_corners = np.array(
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
) * (CUBE_WIDTH / 2)


def get_cube_corner_positions(position, orientation):
    """Get the positions of the cube's corners with the given pose.

    Args:
        position: Position (x, y, z) of the cube.
        orientation: Quaternion (x, y, z, w) with the orientation of the cube.

    Returns:
        (array, shape=(8, 3)): Positions of the corners of the cube in the
            given pose.
    """
    rotation = Rotation(orientation)
    translation = np.asarray(position)

    return rotation.apply(cube_corners) + translation


def sample_goal(difficulty: int, current_state=None):
    # TODO depending on difficulty, the current state of the object needs to be
    # considered (mostly for the orientation)

    # sample uniform position in circle (https://stackoverflow.com/a/50746409)
    radius = max_radius * np.sqrt(random.random())
    theta = random.uniform(0, 2 * np.pi)

    # x,y-position of the cube
    x = radius * np.cos(theta)
    y = radius * np.sin(theta)

    if difficulty == -1 or difficulty == 1:
        # on the ground, random yaw

        z = CUBE_WIDTH / 2
        yaw = random.uniform(0, 2 * np.pi)

        position = np.array((x, y, z))
        orientation = Rotation.from_euler("z", yaw).as_quat()
    else:
        raise ValueError("Invalid difficulty %d" % difficulty)

    return position, orientation


def validate_goal(position, orientation):
    """Validate that the given pose is a valid goal (e.g. no collision)

    Args:
        position:  Goal position.
        orientation:  Goal orientation.

    Returns:
        (bool): True if the given goal pose is within the goal space and does
            not collide with the static environment.
    """
    if len(position) != 3:
        return False, "len(position) != 3"
    if len(orientation) != 4:
        return False, "len(orientation) != 4"
    if np.linalg.norm(position[:2]) > max_radius:
        return False, "Position is outside of the arena circle."
    if position[2] <= min_height:
        return False, "Position is too low."
    if position[2] >= max_height:
        return False, "Position is too high."

    # even if the CoM is above min_height, a corner could be intersecting with
    # the bottom depending on the orientation
    corners = get_cube_corner_positions(position, orientation)
    min_z = min(z for x, y, z in corners)
    if min_z < 0:
        return False, "Position is too low."


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
