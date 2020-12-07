/**
 * @file
 * @brief Access combined log of a TriFinger platform.
 * @copyright 2020, Max Planck Gesellschaft.  All rights reserved.
 * @license BSD 3-clause
 */
#include <robot_fingers/trifinger_platform_log.hpp>

namespace robot_fingers
{
TriFingerPlatformLog::TriFingerPlatformLog(const std::string &robot_log_file,
                                           const std::string &camera_log_file)
    : robot_log_(robot_log_file),
      camera_log_(camera_log_file),
      map_robot_to_camera_index_(robot_log_.data.size(), -1)
{
    if (robot_log_.data.empty())
    {
        throw std::runtime_error("Robot log is empty");
    }
    robot_log_start_index_ = robot_log_.data.front().timeindex;

    // verify that the robot log is complete, i.e. there are no time steps
    // missing
    for (size_t i = 0; i < robot_log_.data.size(); i++)
    {
        if (i + robot_log_start_index_ !=
            static_cast<size_t>(robot_log_.data[i].timeindex))
        {
            throw std::runtime_error("Robot log is incomplete.");
        }
    }

    // for each robot log entry, find the matching camera log entry
    int i_camera = 0;
    for (size_t i_robot = 0; i_robot < robot_log_.data.size(); i_robot++)
    {
        // timestamp in robot log is in seconds, convert to milliseconds
        auto stamp_robot_ms = robot_log_.data[i_robot].timestamp * 1000;

        while (static_cast<size_t>(i_camera) < camera_log_.timestamps.size() &&
               stamp_robot_ms >= camera_log_.timestamps[i_camera])
        {
            i_camera++;
        }
        map_robot_to_camera_index_[i_robot] = i_camera - 1;
    }
}

TriFingerPlatformLog::RobotObservation
TriFingerPlatformLog::get_robot_observation(const time_series::Index &t) const
{
    return robot_log_.data.at(t - robot_log_start_index_).observation;
}

TriFingerPlatformLog::Action TriFingerPlatformLog::get_desired_action(
    const time_series::Index &t) const
{
    return robot_log_.data.at(t - robot_log_start_index_).desired_action;
}

TriFingerPlatformLog::Action TriFingerPlatformLog::get_applied_action(
    const time_series::Index &t) const
{
    return robot_log_.data.at(t - robot_log_start_index_).applied_action;
}

TriFingerPlatformLog::RobotStatus TriFingerPlatformLog::get_robot_status(
    const time_series::Index &t) const
{
    return robot_log_.data.at(t - robot_log_start_index_).status;
}

time_series::Timestamp TriFingerPlatformLog::get_timestamp_ms(
    const time_series::Index &t) const
{
    return robot_log_.data.at(t - robot_log_start_index_).timestamp;
}

TriFingerPlatformLog::CameraObservation
TriFingerPlatformLog::get_camera_observation(const time_series::Index t) const
{
    int i_camera = map_robot_to_camera_index_.at(t - robot_log_start_index_);
    if (i_camera < 0)
    {
        throw std::out_of_range("No camera observation for given time index.");
    }

    return camera_log_.data.at(i_camera);
}

time_series::Index TriFingerPlatformLog::get_first_timeindex() const
{
    return robot_log_start_index_;
}

time_series::Index TriFingerPlatformLog::get_last_timeindex() const
{
    return robot_log_.data.back().timeindex;
}

}  // namespace robot_fingers
