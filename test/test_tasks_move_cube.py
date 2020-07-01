#!/usr/bin/env python3
import random
import unittest
import numpy as np
from scipy.spatial.transform import Rotation

from robot_fingers.tasks import move_cube


class TestMoveCube(unittest.TestCase):
    """Test the functions of the "move cube" task module."""

    def test_get_cube_corner_positions(self):
        # cube half width
        chw = move_cube._CUBE_WIDTH / 2
        # no transformation
        expected_origin_corners = np.array(
            [
                [-chw, -chw, -chw],
                [-chw, -chw, +chw],
                [-chw, +chw, -chw],
                [-chw, +chw, +chw],
                [+chw, -chw, -chw],
                [+chw, -chw, +chw],
                [+chw, +chw, -chw],
                [+chw, +chw, +chw],
            ]
        )
        origin_corners = move_cube.get_cube_corner_positions(
            [0, 0, 0], [0, 0, 0, 1]
        )
        np.testing.assert_array_almost_equal(
            expected_origin_corners, origin_corners
        )

        # only translation
        expected_translated_corners = np.array(
            [
                [-chw + 1, -chw + 2, -chw + 3],
                [-chw + 1, -chw + 2, +chw + 3],
                [-chw + 1, +chw + 2, -chw + 3],
                [-chw + 1, +chw + 2, +chw + 3],
                [+chw + 1, -chw + 2, -chw + 3],
                [+chw + 1, -chw + 2, +chw + 3],
                [+chw + 1, +chw + 2, -chw + 3],
                [+chw + 1, +chw + 2, +chw + 3],
            ]
        )
        translated = move_cube.get_cube_corner_positions(
            [1, 2, 3], [0, 0, 0, 1]
        )
        np.testing.assert_array_almost_equal(
            expected_translated_corners, translated
        )

        # only rotation
        rot_z90 = Rotation.from_euler("z", 90, degrees=True).as_quat()
        expected_rotated_corners = np.array(
            [
                [+chw, -chw, -chw],
                [+chw, -chw, +chw],
                [-chw, -chw, -chw],
                [-chw, -chw, +chw],
                [+chw, +chw, -chw],
                [+chw, +chw, +chw],
                [-chw, +chw, -chw],
                [-chw, +chw, +chw],
            ]
        )
        rotated = move_cube.get_cube_corner_positions([0, 0, 0], rot_z90)
        np.testing.assert_array_almost_equal(expected_rotated_corners, rotated)

        # both rotation and translation
        expected_both_corners = np.array(
            [
                [+chw + 1, -chw + 2, -chw + 3],
                [+chw + 1, -chw + 2, +chw + 3],
                [-chw + 1, -chw + 2, -chw + 3],
                [-chw + 1, -chw + 2, +chw + 3],
                [+chw + 1, +chw + 2, -chw + 3],
                [+chw + 1, +chw + 2, +chw + 3],
                [-chw + 1, +chw + 2, -chw + 3],
                [-chw + 1, +chw + 2, +chw + 3],
            ]
        )
        both = move_cube.get_cube_corner_positions([1, 2, 3], rot_z90)
        np.testing.assert_array_almost_equal(expected_both_corners, both)

    def test_sample_goal_difficulty_1_no_initial_pose(self):
        for i in range(1000):
            goal_pos, goal_rot = move_cube.sample_goal(difficulty=1)
            # verify the goal is valid (i.e. within the allowed ranges)
            try:
                move_cube.validate_goal(goal_pos, goal_rot)
            except move_cube.InvalidGoalError as e:
                self.fail(
                    msg="Invalid goal: {}  pose is {}, {}".format(
                        e, e.position, e.orientation
                    ),
                )

            # verify the goal satisfies conditions of difficulty 1

            # always on ground
            self.assertEqual(goal_pos[2], move_cube._CUBE_WIDTH / 2)

            # no rotations around x or y axes
            rx, ry, rz = Rotation.from_quat(goal_rot).as_euler("xyz")
            self.assertEqual(rx, 0)
            self.assertEqual(ry, 0)

    def test_sample_goal_difficulty_1_with_initial_pose(self):
        for i in range(1000):
            initial_pos, initial_rot = move_cube.sample_goal(-1)

            goal_pos, goal_rot = move_cube.sample_goal(
                difficulty=1,
                current_position=initial_pos,
                current_orientation=initial_rot,
            )

            # verify the goal is valid (i.e. within the allowed ranges)
            try:
                move_cube.validate_goal(goal_pos, goal_rot)
            except move_cube.InvalidGoalError as e:
                self.fail(
                    msg="Invalid goal: {}  pose is {}, {}".format(
                        e, e.position, e.orientation
                    ),
                )

            # verify the goal satisfies conditions of difficulty 1

            # always on ground
            self.assertEqual(goal_pos[2], move_cube._CUBE_WIDTH / 2)

            # no rotations around x or y axes
            expected_rx, expected_ry, expected_rz = Rotation.from_quat(
                goal_rot
            ).as_euler("xyz")
            actual_rx, actual_ry, actual_rz = Rotation.from_quat(
                goal_rot
            ).as_euler("xyz")
            self.assertEqual(actual_rx, expected_rx)
            self.assertEqual(actual_ry, expected_ry)

    def test_sample_goal_difficulty_2_no_initial_pose(self):
        for i in range(1000):
            goal_pos, goal_rot = move_cube.sample_goal(difficulty=2)
            # verify the goal is valid (i.e. within the allowed ranges)
            try:
                move_cube.validate_goal(goal_pos, goal_rot)
            except move_cube.InvalidGoalError as e:
                self.fail(
                    msg="Invalid goal: {}  pose is {}, {}".format(
                        e, e.position, e.orientation
                    ),
                )

            # verify the goal satisfies conditions of difficulty 2

            half_width = move_cube._CUBE_WIDTH / 2
            # TODO max_height +/- half_width?
            self.assertLessEqual(goal_pos[2], move_cube._max_height)
            self.assertGreaterEqual(goal_pos[2], half_width)

            # no rotations around x or y axes
            rx, ry, rz = Rotation.from_quat(goal_rot).as_euler("xyz")
            self.assertEqual(rx, 0)
            self.assertEqual(ry, 0)

    def test_sample_goal_difficulty_2_with_initial_pose(self):
        for i in range(1000):
            initial_pos, initial_rot = move_cube.sample_goal(-1)

            goal_pos, goal_rot = move_cube.sample_goal(
                difficulty=2,
                current_position=initial_pos,
                current_orientation=initial_rot,
            )

            # verify the goal is valid (i.e. within the allowed ranges)
            try:
                move_cube.validate_goal(goal_pos, goal_rot)
            except move_cube.InvalidGoalError as e:
                self.fail(
                    msg="Invalid goal: {}  pose is {}, {}".format(
                        e, e.position, e.orientation
                    ),
                )

            # verify the goal satisfies conditions of difficulty 2

            half_width = move_cube._CUBE_WIDTH / 2
            # TODO max_height +/- half_width?
            self.assertLessEqual(goal_pos[2], move_cube._max_height)
            self.assertGreaterEqual(goal_pos[2], half_width)

            # no rotations around x or y axes
            expected_rx, expected_ry, expected_rz = Rotation.from_quat(
                goal_rot
            ).as_euler("xyz")
            actual_rx, actual_ry, actual_rz = Rotation.from_quat(
                goal_rot
            ).as_euler("xyz")
            self.assertEqual(actual_rx, expected_rx)
            self.assertEqual(actual_ry, expected_ry)

    def test_evaluate_state(self):
        p1 = [0, 0, 0]
        o1 = [0, 0, 0, 1]
        p2 = [1, 2, 3]
        o2 = Rotation.from_euler("z", 0.42).as_quat()

        # needs to be zero for exact match
        cost = move_cube.evaluate_state(p1, o1, p1, o1)
        self.assertEqual(cost, 0)

        # None-zero if there is translation, rotation or both
        self.assertNotEqual(move_cube.evaluate_state(p1, o1, p2, o1), 0)
        self.assertNotEqual(move_cube.evaluate_state(p1, o1, p1, o2), 0)
        self.assertNotEqual(move_cube.evaluate_state(p1, o1, p2, o2), 0)

    def test_validate_goal(self):
        half_width = move_cube._CUBE_WIDTH / 2
        yaw_rotation = Rotation.from_euler("z", 0.42).as_quat()
        full_rotation = Rotation.from_euler("zxz", [0.42, 0.1, -2.3]).as_quat()

        # test some valid goals
        try:
            move_cube.validate_goal([0, 0, half_width], [0, 0, 0, 1])
        except Exception as e:
            self.fail("Valid goal was considered invalid because %s" % e)

        try:
            move_cube.validate_goal([0.05, -0.1, half_width], yaw_rotation)
        except Exception as e:
            self.fail("Valid goal was considered invalid because %s" % e)

        try:
            move_cube.validate_goal([-0.12, 0.0, 0.06], full_rotation)
        except Exception as e:
            self.fail("Valid goal was considered invalid because %s" % e)

        # test some invalid goals

        # invalid values
        with self.assertRaises(ValueError):
            move_cube.validate_goal([0, 0], [0, 0, 0, 1])
        with self.assertRaises(ValueError):
            move_cube.validate_goal([0, 0, 0], [0, 0, 1])

        # invalid positions
        with self.assertRaises(move_cube.InvalidGoalError):
            move_cube.validate_goal([0.3, 0, half_width], [0, 0, 0, 1])
        with self.assertRaises(move_cube.InvalidGoalError):
            move_cube.validate_goal([0, -0.3, half_width], [0, 0, 0, 1])
        with self.assertRaises(move_cube.InvalidGoalError):
            move_cube.validate_goal([0, 0, 0.3], [0, 0, 0, 1])
        with self.assertRaises(move_cube.InvalidGoalError):
            move_cube.validate_goal([0, 0, 0], [0, 0, 0, 1])
        with self.assertRaises(move_cube.InvalidGoalError):
            move_cube.validate_goal([0, 0, -0.01], [0, 0, 0, 1])

        # valid CoM position but rotation makes it reach out of valid range
        with self.assertRaises(move_cube.InvalidGoalError):
            move_cube.validate_goal([0, 0, half_width], full_rotation)


if __name__ == "__main__":
    import rosunit

    rosunit.unitrun(
        "robot_fingers", "test_tasks_move_cube", TestMoveCube,
    )
