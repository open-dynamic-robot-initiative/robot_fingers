/**
 * @file
 * @brief Combined robot/camera driver that replays a given log.
 * @copyright 2021, Max Planck Gesellschaft.  All rights reserved.
 * @license BSD 3-clause
 */
#pragma once

#include <real_time_tools/timer.hpp>
#include <robot_interfaces/finger_types.hpp>
#include <robot_interfaces/monitored_robot_driver.hpp>
#include <robot_interfaces/robot_driver.hpp>
#include <robot_interfaces/sensors/sensor_driver.hpp>

#include <robot_fingers/trifinger_platform_log.hpp>

// FIXME move implementations to cpp file

namespace robot_fingers
{
class TriFingerPlatformLogReplayDriver
    : public robot_interfaces::SensorDriver<
          TriFingerPlatformLog::CameraObservation>,
      public robot_interfaces::RobotDriver<
          TriFingerPlatformLog::Action,
          TriFingerPlatformLog::RobotObservation>
{
public:
    typedef TriFingerPlatformLog::Action Action;
    typedef TriFingerPlatformLog::RobotObservation RobotObservation;
    typedef TriFingerPlatformLog::CameraObservation CameraObservation;

    TriFingerPlatformLogReplayDriver(const std::string &robot_log_file,
                                     const std::string &camera_log_file)
        : robot_interfaces::RobotDriver<Action, RobotObservation>(),
          log_(robot_log_file, camera_log_file),
          t_(0)
    {
    }

    // robot observation
    virtual RobotObservation get_latest_observation() override
    {
        time_series::Index t = t_;
        if (t_ < log_.get_first_timeindex())
        {
            t = log_.get_first_timeindex();
            error_ = "Tried to get observation at t = " + std::to_string(t_) +
                     " but log starts at t = " + std::to_string(t);
        }
        else if (t_ > log_.get_last_timeindex())
        {
            t = log_.get_last_timeindex();
            error_ = "Reached end of log (t = " + std::to_string(t) + ").";
        }

        return log_.get_robot_observation(t);
    }

    Action apply_action(const Action &desired_action) override
    {
        t_++;

        // sleep for 1 ms to replicate timing behaviour of real robot driver
        real_time_tools::Timer::sleep_ms(1.0);

        return desired_action;
    }

    std::string get_error() override
    {
        return error_;
    }

    void shutdown() override
    {
        return;
    }

    void initialize() override
    {
        // reset the time index
        t_ = 0;
    }

    virtual CameraObservation get_observation() override
    {
        // sleep for 100 ms to replicate camera rate of 10 Hz
        real_time_tools::Timer::sleep_ms(100.0);

        // FIXME should check if t_ is valid here

        return log_.get_camera_observation(t_);
    }

private:
    TriFingerPlatformLog log_;
    time_series::Index t_;
    std::string error_;
};

/**
 * @brief Create backend using the log replay driver.
 */
robot_interfaces::TriFingerTypes::BackendPtr create_log_replay_backend(
    std::shared_ptr<TriFingerPlatformLogReplayDriver> driver,
    robot_interfaces::TriFingerTypes::BaseDataPtr robot_data,
    const double first_action_timeout = std::numeric_limits<double>::infinity(),
    const uint32_t max_number_of_actions = 0)
{
    constexpr double MAX_ACTION_DURATION_S = 0.003;
    constexpr double MAX_INTER_ACTION_DURATION_S = 0.005;

    // wrap the actual robot driver directly in a MonitoredRobotDriver
    auto monitored_driver =
        std::make_shared<robot_interfaces::MonitoredRobotDriver<
            TriFingerPlatformLogReplayDriver>>(
            driver, MAX_ACTION_DURATION_S, MAX_INTER_ACTION_DURATION_S);

    constexpr bool real_time_mode = true;
    auto backend = std::make_shared<robot_interfaces::TriFingerTypes::Backend>(
        monitored_driver,
        robot_data,
        real_time_mode,
        first_action_timeout,
        max_number_of_actions);
    backend->set_max_action_repetitions(std::numeric_limits<uint32_t>::max());

    return backend;
}

}  // namespace robot_fingers
