#!/usr/bin/env python3
import unittest
import numpy as np

import robot_interfaces
import robot_fingers.pybullet_drivers
from trifinger_simulation import finger_types_data


class TestPyBulletBackend(unittest.TestCase):
    """Test using pyBullet in the robot interface backend via Python."""

    def _run_position_test(self, finger_type, goal_positions):
        """Run position test for single or tri-finger.

        Moves the robot in a sequence of goal positions in joint-space.  After
        each step, it is checked if the robot reached the goal with some
        tolerance.

        Args:
            finger_type:  Which robot to use.
            goal_positions:  A list of joint goal positions.
        """
        # select the correct types/functions based on which robot is used
        num_fingers = finger_types_data.get_number_of_fingers(finger_type)
        if num_fingers == 1:
            finger_types = robot_interfaces.finger
            create_backend = robot_fingers.pybullet_drivers.create_single_finger_backend
        elif num_fingers == 3:
            finger_types = robot_interfaces.trifinger
            create_backend = robot_fingers.pybullet_drivers.create_trifinger_backend

        robot_data = finger_types.SingleProcessData()

        backend = create_backend(robot_data, real_time_mode=False, visualize=False)

        frontend = finger_types.Frontend(robot_data)
        backend.initialize()

        # Simple example application that moves the finger to random positions.
        for goal in goal_positions:
            action = finger_types.Action(position=goal)
            for _ in range(300):
                t = frontend.append_desired_action(action)
                frontend.wait_until_timeindex(t)

            # check if desired position is reached
            current_position = frontend.get_observation(t).position
            np.testing.assert_array_almost_equal(goal, current_position, decimal=1)

    def test_single_finger_position_control(self):
        """Test position control for the simulated single finger."""
        goals = [
            [0.21, 0.32, -1.10],
            [0.69, 0.78, -1.07],
            [-0.31, 0.24, -0.20],
        ]
        self._run_position_test("fingerone", goals)

    def test_trifinger_position_control(self):
        """Test position control for the simulated TriFinger."""
        # Goals for TriFingerOne
        # goals = [
        #    [0.27, -0.72, -1.03, 0.53, -0.26, -1.67, -0.10, 0.10, -1.36],
        #    [0.38, -0.28, -1.91, 0.22, 0.02, -0.03, 0.00, 0.28, -0.03],
        #    [0.14, 0.27, -0.03, 0.08, -0.08, -0.03, 0.53, -0.00, -0.96],
        # ]
        # self._run_position_test("trifingerone", goals)

        # Goals for TriFingerPro
        goals = [
            [0, 0.9, -1.7, 0, 0.9, -1.7, 0, 0.9, -1.7],
            [-0.05, 0.82, -1.2, -0.06, 0.83, -1.2, -0.07, 0.84, -1.2],
            [0.5, 1.18, -2.39, 0.5, 1.18, -2.4, 0.5, 1.18, -2.4],
        ]
        self._run_position_test("trifingerpro", goals)


if __name__ == "__main__":
    unittest.main()
