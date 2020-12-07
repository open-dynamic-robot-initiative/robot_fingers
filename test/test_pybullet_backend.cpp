/**
 * @file
 * @copyright Copyright (c) 2020, Max Planck Gesellschaft.
 */
#include <gtest/gtest.h>

#include <memory>

#include <robot_interfaces/finger_types.hpp>
#include <robot_interfaces/robot_frontend.hpp>
#include <robot_fingers/pybullet_driver.hpp>

constexpr bool VISUALIZE = false;
constexpr double POSITION_TOLERANCE = 0.05;

// Simple test where the robot is controlled to reach a list of positions and
// after each step it is checked whether that position was actually reached.

TEST(TestPyBulletDriver, monofingerone)
{
    auto robot_data = std::make_shared<
        robot_interfaces::MonoFingerTypes::SingleProcessData>();

    auto backend = trifinger_simulation::create_finger_backend<
        robot_interfaces::MonoFingerTypes,
        trifinger_simulation::PyBulletSingleFingerDriver>(
        robot_data, VISUALIZE, VISUALIZE);

    auto frontend = robot_interfaces::MonoFingerTypes::Frontend(robot_data);

    backend->initialize();

    // Need to release the GIL in the main thread, otherwise the driver
    // (running in the backend thread) is blocked.
    pybind11::gil_scoped_release foo;

    typedef robot_interfaces::MonoFingerTypes::Action Action;

    std::array<Action::Vector, 3> goals;
    goals[0] << 0.21, 0.32, -1.10;
    goals[1] << 0.69, 0.78, -1.07;
    goals[2] << -0.31, 0.24, -0.20;

    for (Action::Vector goal : goals)
    {
        unsigned int t;
        auto action = Action::Position(goal);
        for (int i = 0; i < 500; i++)
        {
            t = frontend.append_desired_action(action);
            frontend.wait_until_timeindex(t);
        }

        // check if desired position is reached
        auto actual_position = frontend.get_observation(t).position;
        // std::cout << actual_position.transpose() << std::endl;
        ASSERT_TRUE(actual_position.isApprox(goal, POSITION_TOLERANCE));
    }
}

TEST(TestPyBulletDriver, trifinger)
{
    auto robot_data =
        std::make_shared<robot_interfaces::TriFingerTypes::SingleProcessData>();

    auto backend = trifinger_simulation::create_finger_backend<
        robot_interfaces::TriFingerTypes,
        trifinger_simulation::PyBulletTriFingerDriver>(
        robot_data, VISUALIZE, VISUALIZE);

    auto frontend = robot_interfaces::TriFingerTypes::Frontend(robot_data);

    backend->initialize();

    // Need to release the GIL in the main thread, otherwise the driver
    // (running in the backend thread) is blocked.
    pybind11::gil_scoped_release foo;

    typedef robot_interfaces::TriFingerTypes::Action Action;

    std::array<Action::Vector, 3> goals;
    goals[0] << 0, 0.9, -1.7, 0, 0.9, -1.7, 0, 0.9, -1.7;
    goals[1] << -0.05,  0.82, -1.2 , -0.06,  0.83, -1.2 , -0.07,  0.84, -1.2;
    goals[2] << 0.5 ,  1.18, -2.39,  0.5 ,  1.18, -2.4 ,  0.5 ,  1.18, -2.4;

    for (Action::Vector goal : goals)
    {
        unsigned int t;
        auto action = Action::Position(goal);
        for (int i = 0; i < 500; i++)
        {
            t = frontend.append_desired_action(action);
            frontend.wait_until_timeindex(t);
        }

        // check if desired position is reached
        auto actual_position = frontend.get_observation(t).position;
        // std::cout << actual_position.transpose() << std::endl;
        ASSERT_TRUE(actual_position.isApprox(goal, POSITION_TOLERANCE))
            << std::setprecision(3)
            << "Expected position: " << goal.transpose()
            << "\nActual position: " << actual_position.transpose();
    }
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
