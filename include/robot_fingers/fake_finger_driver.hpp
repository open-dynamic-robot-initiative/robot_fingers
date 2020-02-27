#pragma once

#include <math.h>
#include <Eigen/Eigen>
#include <blmc_robots/common_header.hpp>
#include <tuple>

#include <blmc_robots/blmc_joint_module.hpp>
#include <blmc_robots/n_joint_blmc_robot_driver.hpp>
#include <robot_interfaces/finger_types.hpp>
#include "real_time_tools/spinner.hpp"
#include "real_time_tools/timer.hpp"

namespace blmc_robots
{
class FakeFingerDriver : public robot_interfaces::RobotDriver<
                             robot_interfaces::FingerTypes::Action,
                             robot_interfaces::FingerTypes::Observation>
{
public:
    typedef robot_interfaces::FingerTypes::Action Action;
    typedef robot_interfaces::FingerTypes::Observation Observation;
    typedef robot_interfaces::FingerTypes::Vector Vector;

    int data_generating_index_ = 0;

    FakeFingerDriver()
    {
    }

    Observation get_latest_observation() override
    {
        // generating observations by a rule to make it easier to check they are
        // being logged correctly as the timeindex increases.

        Observation observation;
        observation.position[0] = data_generating_index_;
        observation.position[1] = 2 * data_generating_index_;
        observation.position[2] = 3 * data_generating_index_;

        observation.velocity[0] = data_generating_index_ + 1;
        observation.velocity[1] = 2 * data_generating_index_ + 1;
        observation.velocity[2] = 3 * data_generating_index_ + 1;

        observation.torque[0] = data_generating_index_ + 2;
        observation.torque[1] = 2 * data_generating_index_ + 2;
        observation.torque[2] = 3 * data_generating_index_ + 2;

        data_generating_index_++;

        return observation;
    }

    Action apply_action(const Action &desired_action) override
    {
        double start_time_sec = real_time_tools::Timer::get_current_time_sec();

        real_time_tools::Timer::sleep_until_sec(start_time_sec + 0.001);

        return desired_action;
    }

    std::string get_error() override
    {
        return "";  // no errors
    }

    void shutdown() override
    {
        return;
    }

    void initialize() override
    {
        return;
    }
};

robot_interfaces::FingerTypes::BackendPtr create_fake_finger_backend(
    robot_interfaces::FingerTypes::DataPtr robot_data)
{
    // adjusted values
    constexpr double MAX_ACTION_DURATION_S = 0.03;
    constexpr double MAX_INTER_ACTION_DURATION_S = 0.05;

    std::shared_ptr<robot_interfaces::RobotDriver<
        robot_interfaces::FingerTypes::Action,
        robot_interfaces::FingerTypes::Observation>>
        robot = std::make_shared<FakeFingerDriver>();

    auto backend = std::make_shared<robot_interfaces::FingerTypes::Backend>(
        robot, robot_data, MAX_ACTION_DURATION_S, MAX_INTER_ACTION_DURATION_S);
    backend->set_max_action_repetitions(-1);

    return backend;
}

}  // namespace blmc_robots
