/**
 * @file
 * @brief Test for processing of robot actions
 * @copyright Copyright (c) 2019, New York University and Max Planck
 *            Gesellschaft.
 */
#include <gtest/gtest.h>
#include <robot_fingers/n_joint_blmc_robot_driver.hpp>
#include <robot_interfaces/n_joint_robot_types.hpp>

/**
 * @brief Fixture for the tests of process_desired_action().
 */
class TestProcessDesiredAction : public ::testing::Test
{
protected:
    using Types = robot_interfaces::SimpleNJointRobotTypes<2>;
    using Driver = blmc_robots::SimpleNJointBlmcRobotDriver<2>;
    using Vector = Types::Action::Vector;

    Types::Observation observation;
    double max_torque_Nm;
    Vector safety_kd;
    Vector default_position_control_kp;
    Vector default_position_control_kd;

    void SetUp() override
    {
        // set some arbitrary default values.  Tests can overwrite specific
        // values if needed.
        observation.position << 12.34, -0.42;
        observation.velocity << .4, -.2;
        observation.torque << 0.3, -0.25;

        max_torque_Nm = 0.3;
        safety_kd << 0.1, 0.1;

        default_position_control_kp << 3, 4;
        default_position_control_kd << .3, .4;
    }
};

/**
 * @brief Test if a valid torque command is returned unchanged (no safety).
 */
TEST_F(TestProcessDesiredAction, valid_torque_no_safety)
{
    // disable velocity damping for this test
    safety_kd << 0., 0.;

    Vector desired_torque;
    desired_torque << 0.1, 0.2;
    Types::Action action = Types::Action::Torque(desired_torque);

    Types::Action resulting_action =
        Driver::process_desired_action(action,
                                       observation,
                                       max_torque_Nm,
                                       safety_kd,
                                       default_position_control_kp,
                                       default_position_control_kd);

    ASSERT_EQ(desired_torque[0], resulting_action.torque[0]);
    ASSERT_EQ(desired_torque[1], resulting_action.torque[1]);
}

/**
 * @brief Test if clamping to max. torque is working.
 */
TEST_F(TestProcessDesiredAction, exceed_max_torque_no_safety)
{
    // disable velocity damping for this test
    safety_kd << 0., 0.;

    Vector desired_torque;
    desired_torque << 0.5, -1.2;
    Types::Action action = Types::Action::Torque(desired_torque);

    Types::Action resulting_action =
        Driver::process_desired_action(action,
                                       observation,
                                       max_torque_Nm,
                                       safety_kd,
                                       default_position_control_kp,
                                       default_position_control_kd);

    ASSERT_EQ(max_torque_Nm, resulting_action.torque[0]);
    ASSERT_EQ(-max_torque_Nm, resulting_action.torque[1]);
}

/**
 * @brief Test velocity damping with low velocity
 */
TEST_F(TestProcessDesiredAction, velocity_damping_low_velocity)
{
    Vector desired_torque;
    desired_torque << 0.1, 0.2;
    Types::Action action = Types::Action::Torque(desired_torque);

    Types::Action resulting_action =
        Driver::process_desired_action(action,
                                       observation,
                                       max_torque_Nm,
                                       safety_kd,
                                       default_position_control_kp,
                                       default_position_control_kd);

    // joint 0 has positive velocity so expecting a reduced torque.
    // joint 1 has negative velocity so expecting an increased torque.
    // both should remain within the max. torque range.
    ASSERT_LT(resulting_action.torque[0], desired_torque[0]);
    ASSERT_GE(resulting_action.torque[0], -max_torque_Nm);
    ASSERT_GT(resulting_action.torque[1], desired_torque[1]);
    ASSERT_LE(resulting_action.torque[1], max_torque_Nm);
}

/**
 * @brief Test velocity damping with very high velocity.
 *
 * This test ensures that torques cannot exceed the allowed max. torque due to
 * the velocity damping.
 */
TEST_F(TestProcessDesiredAction, velocity_damping_high_velocity)
{
    // set very high velocity
    observation.velocity << 4000, -2000;

    Vector desired_torque;
    desired_torque << 0.1, 0.2;
    Types::Action action = Types::Action::Torque(desired_torque);

    Types::Action resulting_action =
        Driver::process_desired_action(action,
                                       observation,
                                       max_torque_Nm,
                                       safety_kd,
                                       default_position_control_kp,
                                       default_position_control_kd);

    // joint 0 has positive velocity so expecting a reduced torque.
    // joint 1 has negative velocity so expecting an increased torque.
    // both should remain within the max. torque range.
    ASSERT_LT(resulting_action.torque[0], desired_torque[0]);
    ASSERT_GE(resulting_action.torque[0], -max_torque_Nm);
    ASSERT_GT(resulting_action.torque[1], desired_torque[1]);
    ASSERT_LE(resulting_action.torque[1], max_torque_Nm);
}

/**
 * @brief Test basic position controller (default gains, no velocity)
 */
TEST_F(TestProcessDesiredAction, position_controller_basic)
{
    // disable velocity damping for this test
    safety_kd << 0., 0.;

    observation.position << 0, 0;
    observation.velocity << 0, 0;

    Vector desired_position;
    desired_position << -1, 1;
    Types::Action action = Types::Action::Position(desired_position);

    Types::Action resulting_action =
        Driver::process_desired_action(action,
                                       observation,
                                       max_torque_Nm,
                                       safety_kd,
                                       default_position_control_kp,
                                       default_position_control_kd);

    // Just testing here if the torque command goes in the right direction

    ASSERT_LT(resulting_action.torque[0], 0.0);
    ASSERT_GE(resulting_action.torque[0], -max_torque_Nm);

    ASSERT_GT(resulting_action.torque[1], 0.0);
    ASSERT_LE(resulting_action.torque[1], max_torque_Nm);

    // verify that position and gains are set correctly in the returned action
    ASSERT_EQ(desired_position[0], resulting_action.position[0]);
    ASSERT_EQ(desired_position[1], resulting_action.position[1]);
    ASSERT_EQ(default_position_control_kp[0], resulting_action.position_kp[0]);
    ASSERT_EQ(default_position_control_kp[1], resulting_action.position_kp[1]);
    ASSERT_EQ(default_position_control_kd[0], resulting_action.position_kd[0]);
    ASSERT_EQ(default_position_control_kd[1], resulting_action.position_kd[1]);
}

/**
 * @brief Test position controller for only one joint.
 */
TEST_F(TestProcessDesiredAction, position_controller_one_joint_only)
{
    // disable velocity damping for this test
    safety_kd << 0., 0.;

    observation.position << 0, 0;
    observation.velocity << 0, 0;

    Vector desired_position;
    desired_position << -1, std::numeric_limits<double>::quiet_NaN();
    Types::Action action = Types::Action::Position(desired_position);

    Types::Action resulting_action =
        Driver::process_desired_action(action,
                                       observation,
                                       max_torque_Nm,
                                       safety_kd,
                                       default_position_control_kp,
                                       default_position_control_kd);

    // Just testing here if the torque command goes in the right direction
    ASSERT_LT(resulting_action.torque[0], 0.0);
    ASSERT_GE(resulting_action.torque[0], -max_torque_Nm);

    // desired position for joint 1 is nan, so no torque should be generated
    ASSERT_EQ(0.0, resulting_action.torque[1]);
}

/**
 * @brief Test position control with velocity (i.e. verifying D-gain is working)
 */
TEST_F(TestProcessDesiredAction, position_controller_with_velocity)
{
    // disable velocity damping for this test
    safety_kd << 0., 0.;

    observation.position << 0, 0;
    observation.velocity << 0, 0;

    Vector desired_position;
    // use low position change to not saturate torques
    desired_position << -.01, .01;

    Types::Action result_action_without_velocity =
        Driver::process_desired_action(
            Types::Action::Position(desired_position),
            observation,
            max_torque_Nm,
            safety_kd,
            default_position_control_kp,
            default_position_control_kd);

    observation.velocity << -.01, .01;

    Types::Action result_action_with_velocity = Driver::process_desired_action(
        Types::Action::Position(desired_position),
        observation,
        max_torque_Nm,
        safety_kd,
        default_position_control_kp,
        default_position_control_kd);

    // Since velocity has same sign as position error, the resulting
    // (absolute) torques should be lower (since it is
    //   torque = P * position_error - D * velocity )
    EXPECT_LT(result_action_without_velocity.torque[0],
              result_action_with_velocity.torque[0]);
    EXPECT_GT(result_action_without_velocity.torque[1],
              result_action_with_velocity.torque[1]);
}

/**
 * @brief Test position control with custom gains.
 */
TEST_F(TestProcessDesiredAction, position_controller_custom_gains)
{
    // disable velocity damping for this test
    safety_kd << 0., 0.;

    observation.position << 0, 0;
    observation.velocity << 0, 0;

    Vector desired_position, kp, kd;
    // use low position change to not saturate torques with lower gains
    desired_position << -.01, .01;
    // set gains significantly higher than the default gains above
    kp << 10, 10;
    kd << 5, 5;

    Types::Action resulting_action_default_gains =
        Driver::process_desired_action(
            Types::Action::Position(desired_position),
            observation,
            max_torque_Nm,
            safety_kd,
            default_position_control_kp,
            default_position_control_kd);

    Types::Action resulting_action_custom_gains =
        Driver::process_desired_action(
            Types::Action::Position(desired_position, kp, kd),
            observation,
            max_torque_Nm,
            safety_kd,
            default_position_control_kp,
            default_position_control_kd);

    // verify that custom gains are set in resulting action
    EXPECT_EQ(kp[0], resulting_action_custom_gains.position_kp[0]);
    EXPECT_EQ(kp[1], resulting_action_custom_gains.position_kp[1]);
    EXPECT_EQ(kd[0], resulting_action_custom_gains.position_kd[0]);
    EXPECT_EQ(kd[1], resulting_action_custom_gains.position_kd[1]);

    // Since custom gains are higher, the resulting (absolute) torques should be
    // higher
    EXPECT_GT(resulting_action_default_gains.torque[0],
              resulting_action_custom_gains.torque[0]);
    EXPECT_LT(resulting_action_default_gains.torque[1],
              resulting_action_custom_gains.torque[1]);
}

/**
 * @brief Test combined torque and position control.
 */
TEST_F(TestProcessDesiredAction, position_controller_and_torque)
{
    // disable velocity damping for this test
    safety_kd << 0., 0.;

    observation.position << 0, 0;
    observation.velocity << 0, 0;

    Vector desired_torque;
    desired_torque << -0.2, 0.2;
    Vector desired_position;
    desired_position << -.1, .1;
    Types::Action action =
        Types::Action::TorqueAndPosition(desired_torque, desired_position);

    Types::Action resulting_action =
        Driver::process_desired_action(action,
                                       observation,
                                       max_torque_Nm,
                                       safety_kd,
                                       default_position_control_kp,
                                       default_position_control_kd);

    // Just testing here if the torque command goes in the right direction

    ASSERT_LT(resulting_action.torque[0], desired_torque[0]);
    ASSERT_GE(resulting_action.torque[0], -max_torque_Nm);

    ASSERT_GT(resulting_action.torque[1], desired_torque[1]);
    ASSERT_LE(resulting_action.torque[1], max_torque_Nm);
}
