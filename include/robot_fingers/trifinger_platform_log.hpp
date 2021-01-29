/**
 * @file
 * @brief Access combined log of a TriFinger platform.
 * @copyright 2020, Max Planck Gesellschaft.  All rights reserved.
 * @license BSD 3-clause
 */
#pragma once

#include <robot_interfaces/finger_types.hpp>
#include <robot_interfaces/sensors/sensor_log_reader.hpp>
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
class TriFingerPlatformLog
{
public:
    // typedefs for easy access
    typedef robot_interfaces::TriFingerTypes::Action Action;
    typedef robot_interfaces::TriFingerTypes::Observation RobotObservation;
    typedef robot_interfaces::Status RobotStatus;
    typedef trifinger_object_tracking::TriCameraObjectObservation
        CameraObservation;

    TriFingerPlatformLog(const std::string &robot_log_file,
                         const std::string &camera_log_file);

    /**
     * @brief Get robot observation of the time step t.
     */
    RobotObservation get_robot_observation(const time_series::Index &t) const;

    /**
     * @brief Get desired action of time step t.
     */
    Action get_desired_action(const time_series::Index &t) const;

    /**
     * @brief Get actually applied action of time step t.
     */
    Action get_applied_action(const time_series::Index &t) const;

    /**
     * @brief Get robot status of time step t.
     */
    RobotStatus get_robot_status(const time_series::Index &t) const;

    /**
     * @brief Get timestamp (in milliseconds) of time step t.
     */
    time_series::Timestamp get_timestamp_ms(const time_series::Index &t) const;

    /**
     * @brief Get camera images of time step t.
     *
     * @param t  Time index of the robot time series.  This is internally
     *      mapped to the corresponding time index of the camera time series.
     *
     * @return Camera images of time step t.
     */
    CameraObservation get_camera_observation(const time_series::Index t) const;

    //! @brief Get the time index of the first time step in the log.
    time_series::Index get_first_timeindex() const;

    //! @brief Get the time index of the last time step in the log.
    time_series::Index get_last_timeindex() const;

private:
    robot_interfaces::TriFingerTypes::BinaryLogReader robot_log_;
    robot_interfaces::SensorLogReader<CameraObservation> camera_log_;

    time_series::Index robot_log_start_index_;

    std::vector<int> map_robot_to_camera_index_;
};
}  // namespace robot_fingers
