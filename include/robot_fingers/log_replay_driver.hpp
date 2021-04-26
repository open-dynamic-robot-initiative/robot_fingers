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
          log_(robot_log_file, camera_log_file)
    {
    }

    // robot observation
    virtual RobotObservation get_latest_observation() override
    {
        time_series::Index t = t_;
        // If the current time step is not contained in the log, return the
        // first/last message of the log to have some meaningful return value
        // but also set an error.  This will cause the robot back end to
        // shut down.
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

        // set the "current" timestamp w.r.t. the log (used to determine when to
        // publish the next camera observation)
        robot_timestamp_ms_ = log_.get_timestamp_ms(t);

        return log_.get_robot_observation(t);
    }

    Action apply_action(const Action &desired_action) override
    {
        t_++;

        // sleep for 1 ms to replicate timing behaviour of the robot
        real_time_tools::Timer::sleep_ms(1.0);

        return desired_action;
    }

    std::string get_error() override
    {
        return error_;
    }

    void shutdown() override
    {
        is_shutdown_requested_ = true;
    }

    void initialize() override
    {
        // nothing to do here
        return;
    }

    virtual CameraObservation get_observation() override
    {
        static int t =
            log_.map_robot_to_camera_index(log_.get_first_timeindex());
        static bool first_call = true;

        // NOTE: log_.get_raw_camera_log().timestamps[t] is in ms

        // Sleep until it's time to provide the next camera observation (using
        // the robot log timestamp as reference)
        // However, do not sleep in the first call of this method as the first
        // camera observation should be provided immediately.
        while (!is_shutdown_requested_ and !first_call and
               robot_timestamp_ms_ < log_.get_raw_camera_log().timestamps[t])
        {
            real_time_tools::Timer::sleep_ms(1.0);
        }
        // TODO: the above results in no sleep happening anymore once the robot
        // log reached it's end.  This is not really an issue but a bit ugly.

        CameraObservation observation = log_.get_raw_camera_log().data.at(t);

        // adjust frame timestamps so they roughly match with the observation
        // timestamp (which is created when adding the observation to the time
        // series)
        double time_offset_ms = real_time_tools::Timer::get_current_time_ms() -
                                log_.get_raw_camera_log().timestamps[t];
        for (auto &frame : observation.cameras)
        {
            frame.timestamp += time_offset_ms / 1000.0;
        }

        if (t_ < log_.get_last_timeindex() &&
            t < log_.get_raw_camera_log().data.size() - 1)
        {
            t++;
        }

        first_call = false;

        return observation;
    }

private:
    TriFingerPlatformLog log_;
    time_series::Index t_ = 0;
    double robot_timestamp_ms_ = 0.0;  // s or ms?
    std::string error_;

    std::atomic<bool> is_shutdown_requested_ = false;
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
