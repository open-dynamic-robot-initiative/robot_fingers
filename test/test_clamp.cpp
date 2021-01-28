/**
 * @file
 * @brief Tests for the clamp function
 * @copyright Copyright (c) 2020, New York University & Max Planck Gesellschaft.
 */
#include <gtest/gtest.h>
#include <Eigen/Eigen>

#include <robot_fingers/clamp.hpp>

TEST(TestClamp, clamp_within)
{
    Eigen::Vector3d vec(1, 2, 3);
    Eigen::Vector3d clamped = robot_fingers::clamp(vec, 0, 4);
    Eigen::Vector3d expected(1, 2, 3);
    ASSERT_EQ(expected, clamped);
}

TEST(TestClamp, clamp_below)
{
    Eigen::Vector3d vec(1, 2, 3);
    Eigen::Vector3d clamped = robot_fingers::clamp(vec, 2, 4);
    Eigen::Vector3d expected(2, 2, 3);
    ASSERT_EQ(expected, clamped);
}

TEST(TestClamp, clamp_above)
{
    Eigen::Vector4d vec(1, 2, 3, 4);
    Eigen::Vector4d clamped = robot_fingers::clamp(vec, 1, 3);
    Eigen::Vector4d expected(1, 2, 3, 3);
    ASSERT_EQ(expected, clamped);
}

TEST(TestClamp, clamp_below_and_above)
{
    Eigen::Vector4d vec(1, 2, 3, 4);
    Eigen::Vector4d clamped = robot_fingers::clamp(vec, 2, 3);
    Eigen::Vector4d expected(2, 2, 3, 3);
    ASSERT_EQ(expected, clamped);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
