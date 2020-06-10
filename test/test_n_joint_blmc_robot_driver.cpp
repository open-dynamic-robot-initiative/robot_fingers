/**
 * @file
 * @brief Test for NJointBlmcRobotDriver.
 * @copyright Copyright (c) 2020, New York University & Max Planck Gesellschaft.
 */
#include <gtest/gtest.h>
#include <robot_fingers/n_joint_blmc_robot_driver.hpp>

using Driver = blmc_robots::SimpleNJointBlmcRobotDriver<2>;

TEST(TestNJointBlmcRobotDriverConfig, is_within_joint_limits)
{
    Driver::Config config;
    config.hard_position_limits_lower << -1.0, 0;
    config.hard_position_limits_upper << +0.5, +1.0;

    // bounds are inclusive
    ASSERT_TRUE(config.is_within_hard_position_limits(
        config.hard_position_limits_lower));
    ASSERT_TRUE(config.is_within_hard_position_limits(
        config.hard_position_limits_upper));

    // some positions inside the limits
    ASSERT_TRUE(config.is_within_hard_position_limits(Driver::Vector(0, 0.3)));
    ASSERT_TRUE(
        config.is_within_hard_position_limits(Driver::Vector(-0.9, 0.9)));
    ASSERT_TRUE(
        config.is_within_hard_position_limits(Driver::Vector(0.4, 0.1)));

    // some positions outside the limits
    ASSERT_FALSE(config.is_within_hard_position_limits(Driver::Vector(-2, -1)));
    ASSERT_FALSE(
        config.is_within_hard_position_limits(Driver::Vector(-2, 0.5)));
    ASSERT_FALSE(
        config.is_within_hard_position_limits(Driver::Vector(-0.5, -1)));
    ASSERT_FALSE(
        config.is_within_hard_position_limits(Driver::Vector(-0.5, 2)));
    ASSERT_FALSE(config.is_within_hard_position_limits(Driver::Vector(1, 2)));
    ASSERT_FALSE(config.is_within_hard_position_limits(Driver::Vector(1, 0.5)));
}
