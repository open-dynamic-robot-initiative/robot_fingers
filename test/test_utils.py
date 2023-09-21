import pytest
import numpy as np

from robot_fingers import utils


def test_min_jerk_trajectory():
    start = [0.0, 1.0]
    end = [2.0, 2.0]
    steps = 5
    expected_steps = [
        [0.0, 1.0],
        [0.11584, 1.05792],
        [0.63488, 1.31744],
        [1.36512, 1.68256],
        [1.88416, 1.94208],
        [2.0, 2.0],
    ]

    traj = utils.min_jerk_trajectory(start, end, steps)
    np.testing.assert_array_almost_equal(expected_steps[0], next(traj))
    np.testing.assert_array_almost_equal(expected_steps[1], next(traj))
    np.testing.assert_array_almost_equal(expected_steps[2], next(traj))
    np.testing.assert_array_almost_equal(expected_steps[3], next(traj))
    np.testing.assert_array_almost_equal(expected_steps[4], next(traj))
    with pytest.raises(StopIteration):
        next(traj)

    # try again but using numpy arrays as input
    traj = utils.min_jerk_trajectory(np.array(start), np.array(end), steps)
    np.testing.assert_array_almost_equal(expected_steps[0], next(traj))
    np.testing.assert_array_almost_equal(expected_steps[1], next(traj))
    np.testing.assert_array_almost_equal(expected_steps[2], next(traj))
    np.testing.assert_array_almost_equal(expected_steps[3], next(traj))
    np.testing.assert_array_almost_equal(expected_steps[4], next(traj))
    with pytest.raises(StopIteration):
        next(traj)


def test_min_jerk_trajectory__invalid_inputs():
    with pytest.raises(ValueError, match="num_steps must be >= 1"):
        next(utils.min_jerk_trajectory([0, 0], [1, 1], 0))

    with pytest.raises(ValueError, match="num_steps must be >= 1"):
        next(utils.min_jerk_trajectory([0, 0], [1, 1], -10))

    with pytest.raises(ValueError, match="shapes"):
        next(utils.min_jerk_trajectory([0, 0], [1, 1, 1], 10))
