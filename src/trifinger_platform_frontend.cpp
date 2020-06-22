/**
 * @file
 * @brief TODO
 * @copyright 2020, Max Planck Gesellschaft.  All rights reserved.
 * @license BSD 3-clause
 */
#include <robot_fingers/trifinger_platform_frontend.hpp>

namespace robot_fingers
{
TriFingerPlatformFrontend::TriFingerPlatformFrontend(
    robot_interfaces::TriFingerTypes::BaseDataPtr robot_data,
    trifinger_object_tracking::ObjectTrackerData::Ptr object_tracker_data,
    std::shared_ptr<robot_interfaces::SensorData<CameraObservation>>
        camera_data)
    : robot_frontend_(robot_data),
      object_tracker_frontend_(object_tracker_data),
      camera_frontend_(camera_data)
{
}

TriFingerPlatformFrontend::TriFingerPlatformFrontend()
    : robot_frontend_(
          std::make_shared<robot_interfaces::TriFingerTypes::MultiProcessData>(
              "trifinger", false)),
      object_tracker_frontend_(
          std::make_shared<trifinger_object_tracking::ObjectTrackerData>(
              "object_tracker", false)),
      camera_frontend_(
          std::make_shared<
              robot_interfaces::MultiProcessSensorData<CameraObservation>>(
              "tricamera", false, 10))

{
}

time_series::Index TriFingerPlatformFrontend::append_desired_action(
    const Action &desired_action)
{
    return robot_frontend_.append_desired_action(desired_action);
}

TriFingerPlatformFrontend::RobotObservation
TriFingerPlatformFrontend::get_robot_observation(
    const time_series::Index &t) const
{
    return robot_frontend_.get_observation(t);
}

TriFingerPlatformFrontend::Action TriFingerPlatformFrontend::get_desired_action(
    const time_series::Index &t) const
{
    return robot_frontend_.get_desired_action(t);
}

TriFingerPlatformFrontend::Action TriFingerPlatformFrontend::get_applied_action(
    const time_series::Index &t) const
{
    return robot_frontend_.get_applied_action(t);
}

TriFingerPlatformFrontend::RobotStatus
TriFingerPlatformFrontend::get_robot_status(const time_series::Index &t) const
{
    return robot_frontend_.get_status(t);
}

time_series::Timestamp TriFingerPlatformFrontend::get_timestamp_ms(
    const time_series::Index &t) const
{
    return robot_frontend_.get_timestamp_ms(t);
}

time_series::Index TriFingerPlatformFrontend::get_current_timeindex() const
{
    return robot_frontend_.get_current_timeindex();
}

void TriFingerPlatformFrontend::wait_until_timeindex(
    const time_series::Index &t) const
{
    robot_frontend_.wait_until_timeindex(t);
}

trifinger_object_tracking::ObjectPose
TriFingerPlatformFrontend::get_object_pose(const time_series::Index t) const
{
    auto t_tracker = find_matching_timeindex(object_tracker_frontend_, t);
    return object_tracker_frontend_.get_pose(t_tracker);
}

TriFingerPlatformFrontend::CameraObservation
TriFingerPlatformFrontend::get_camera_observation(
    const time_series::Index t) const
{
    auto t_camera = find_matching_timeindex(camera_frontend_, t);
    return camera_frontend_.get_observation(t_camera);
}

}  // namespace robot_fingers
