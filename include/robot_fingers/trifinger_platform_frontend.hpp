/**
 * @file
 * @brief TODO
 * @copyright 2020, Max Planck Gesellschaft.  All rights reserved.
 * @license BSD 3-clause
 */
#pragma once

// TODO move to a separate package to not add unnecessary dependencies to
// robot_fingers?
#include <robot_interfaces/finger_types.hpp>
#include <robot_interfaces/sensors/sensor_frontend.hpp>
#include <trifinger_cameras/tricamera_observation.hpp>
#include <trifinger_object_tracking/object_tracker_frontend.hpp>

namespace robot_fingers
{
// TODO move implementation to cpp file
// NOTE: Inheriting from Frontend makes implementation easier but means that we
// cannot rename get_observation
class TriFingerPlatformFrontend
    : public robot_interfaces::TriFingerTypes::Frontend
{
public:
    // typedefs for easy access
    typedef robot_interfaces::TriFingerTypes::Action Action;
    typedef robot_interfaces::TriFingerTypes::Observation RobotObservation;
    typedef robot_interfaces::Status Status;
    typedef trifinger_cameras::TriCameraObservation CameraObservation;

    TriFingerPlatformFrontend(
        robot_interfaces::TriFingerTypes::BaseDataPtr robot_data,
        trifinger_object_tracking::ObjectTrackerData::Ptr object_tracker_data,
        std::shared_ptr<robot_interfaces::SensorData<CameraObservation>>
            camera_data);

    TriFingerPlatformFrontend();

    // alias
    RobotObservation get_robot_observation(const time_series::Index t) const;

    trifinger_object_tracking::ObjectPose get_object_pose(
        const time_series::Index t) const;

    CameraObservation get_camera_observation(const time_series::Index t) const;

private:
    trifinger_object_tracking::ObjectTrackerFrontend object_tracker_frontend_;
    robot_interfaces::SensorFrontend<CameraObservation> camera_frontend_;
};
}  // namespace robot_fingers
