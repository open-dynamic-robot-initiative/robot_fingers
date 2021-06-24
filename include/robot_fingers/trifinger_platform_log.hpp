/**
 * @file
 * @brief Access combined log of a TriFinger platform.
 * @copyright 2020, Max Planck Gesellschaft.  All rights reserved.
 * @license BSD 3-clause
 */
#pragma once

#include <robot_interfaces/finger_types.hpp>
#include <robot_interfaces/sensors/sensor_log_reader.hpp>
#include <trifinger_cameras/tricamera_observation.hpp>
#include <trifinger_object_tracking/tricamera_object_observation.hpp>

namespace robot_fingers
{
/**
 * @brief Load robot and camera log and match observations like during runtime.
 *
 * The robot and camera observations are provided asynchronously.  To access
 * both through a common time index, the @ref TriFingerPlatformFrontend class
 * maps "robot time indices" to the corresponding camera observations based on
 * the time stamps.  This mapping is not explicitly saved in the log files.
 * Therefore, the TriFingerPlatformLog class provides an interface to load
 * robot and camera logs together and performs the mapping from robot to camera
 * time index in the same way as it is happening in @ref
 * TriFingerPlatformFrontend.
 */
template <typename CameraObservation_t>
class T_TriFingerPlatformLog
{
public:
    // typedefs for easy access
    typedef robot_interfaces::TriFingerTypes::Action Action;
    typedef robot_interfaces::TriFingerTypes::Observation RobotObservation;
    typedef robot_interfaces::Status RobotStatus;
    typedef CameraObservation_t CameraObservation;

    T_TriFingerPlatformLog(const std::string &robot_log_file,
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

            while (static_cast<size_t>(i_camera) <
                       camera_log_.timestamps.size() &&
                   stamp_robot_ms >= camera_log_.timestamps[i_camera])
            {
                i_camera++;
            }
            map_robot_to_camera_index_[i_robot] = i_camera - 1;
        }
    }

    /**
     * @brief Access the robot log.
     */
    const robot_interfaces::TriFingerTypes::BinaryLogReader &get_robot_log()
        const
    {
        return robot_log_;
    }

    /**
     * @brief Access the camera log.
     */
    const robot_interfaces::SensorLogReader<CameraObservation> &get_camera_log()
        const
    {
        return camera_log_;
    }

    /**
     * @brief Access the index mapping from robot to camera log.
     *
     * Note that the robot observation index does not necessarily match with the
     * time index!
     */
    const std::vector<int> &get_map_robot_to_camera_index() const
    {
        return map_robot_to_camera_index_;
    }

    /**
     * @brief Get robot observation of the time step t.
     */
    RobotObservation get_robot_observation(const time_series::Index &t) const
    {
        return robot_log_.data.at(t - robot_log_start_index_).observation;
    }

    /**
     * @brief Get desired action of time step t.
     */
    Action get_desired_action(const time_series::Index &t) const
    {
        return robot_log_.data.at(t - robot_log_start_index_).desired_action;
    }

    /**
     * @brief Get actually applied action of time step t.
     */
    Action get_applied_action(const time_series::Index &t) const
    {
        return robot_log_.data.at(t - robot_log_start_index_).applied_action;
    }

    /**
     * @brief Get robot status of time step t.
     */
    RobotStatus get_robot_status(const time_series::Index &t) const
    {
        return robot_log_.data.at(t - robot_log_start_index_).status;
    }

    /**
     * @brief Get timestamp (in milliseconds) of time step t.
     */
    time_series::Timestamp get_timestamp_ms(const time_series::Index &t) const
    {
        // the timestamp in the log is in seconds
        return robot_log_.data.at(t - robot_log_start_index_).timestamp *
               1000.0;
    }

    /**
     * @brief Get camera images of time step t.
     *
     * @param t  Time index of the robot time series.  This is internally
     *      mapped to the corresponding time index of the camera time series.
     *
     * @return Camera images of time step t.
     */
    CameraObservation get_camera_observation(const time_series::Index t) const
    {
        int i_camera =
            map_robot_to_camera_index_.at(t - robot_log_start_index_);
        if (i_camera < 0)
        {
            throw std::out_of_range(
                "No camera observation for given time index.");
        }

        return camera_log_.data.at(i_camera);
    }

    //! @brief Get the time index of the first time step in the log.
    time_series::Index get_first_timeindex() const
    {
        return robot_log_start_index_;
    }

    //! @brief Get the time index of the last time step in the log.
    time_series::Index get_last_timeindex() const
    {
        return robot_log_.data.back().timeindex;
    }

private:
    robot_interfaces::TriFingerTypes::BinaryLogReader robot_log_;
    robot_interfaces::SensorLogReader<CameraObservation> camera_log_;

    time_series::Index robot_log_start_index_;

    std::vector<int> map_robot_to_camera_index_;
};

// typedefs for easier use
typedef T_TriFingerPlatformLog<trifinger_cameras::TriCameraObservation>
    TriFingerPlatformLog;
typedef T_TriFingerPlatformLog<
    trifinger_object_tracking::TriCameraObjectObservation>
    TriFingerPlatformWithObjectLog;
}  // namespace robot_fingers
