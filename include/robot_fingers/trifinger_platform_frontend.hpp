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
class TriFingerPlatformFrontend : public robot_interfaces::TriFingerTypes::Frontend
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
            camera_data)
        : robot_interfaces::TriFingerTypes::Frontend(robot_data),
          object_tracker_frontend_(object_tracker_data),
          camera_frontend_(camera_data)
    {
    }

    TriFingerPlatformFrontend()
        : robot_interfaces::TriFingerTypes::Frontend(
              std::make_shared<
                  robot_interfaces::TriFingerTypes::MultiProcessData>(
                  "trifinger", false)),
          object_tracker_frontend_(
              std::make_shared<trifinger_object_tracking::ObjectTrackerData>(
                  "object_tracker", false)),
          camera_frontend_(
              std::make_shared<robot_interfaces::MultiProcessSensorData<CameraObservation>>(
                  "tricamera", false, 10))

    {
    }

    // alias
    RobotObservation get_robot_observation(const time_series::Index t) const
    {
        return get_observation(t);
    }

    trifinger_object_tracking::ObjectPose get_object_pose(
        const time_series::Index t) const
    {
        // The given time index t refers to the robot data time series.  To
        // provide the correct object tracker observation for this time step,
        // find the highest time index t_o of the object tracker where
        //
        //     timestamp(t_o) <= timestamp(t)
        //
        // Note that this is not always the one that is closest w.r.t. to the
        // timestamp, i.e.
        //
        //     t_o != argmin(|timestamp(t_o) - timestamp(t)|)
        //
        // The latter would not be deterministic: the outcome could change when
        // called twice with the same `t` if a new object tracker observation
        // arrived in between the calls.

        // TODO: The implementation below is very naive.
        // It simply does a linear search starting from the latest time index.
        // So worst case performance is O(n) where n is the number of object
        // tracker observations over the period that is covered by the buffer of
        // the robot data.
        //
        // Options to speed this up:
        // - binary search (?)
        // - estimate time step size for tracker based on last observations
        // - store matched indices of last call
        //
        // Note, however, that `t` is very likely the latest time index in most
        // cases.  In this case the match for `t_o` will also be the latest
        // index of the corresponding time series.  In this case, the complexity
        // is O(1).  So even when implementing a more complex search algorithm,
        // the first candidate for `t_o` that is checked should always be the
        // latest one.

        time_series::Timestamp stamp_robot = get_timestamp_ms(t);

        time_series::Index t_tracker =
            object_tracker_frontend_.get_current_timeindex();
        time_series::Timestamp stamp_tracker =
            object_tracker_frontend_.get_timestamp_ms(t_tracker);

        while (stamp_robot < stamp_tracker)
        {
            t_tracker--;
            stamp_tracker =
                object_tracker_frontend_.get_timestamp_ms(t_tracker);
        }

        return object_tracker_frontend_.get_pose(t_tracker);
    }

    CameraObservation get_camera_observation(
        const time_series::Index t) const
    {
        // FIXME redundancy with get_object_pose()

        time_series::Timestamp stamp_robot = get_timestamp_ms(t);

        time_series::Index t_camera =
            camera_frontend_.get_current_timeindex();
        time_series::Timestamp stamp_camera =
            camera_frontend_.get_timestamp_ms(t_camera);

        while (stamp_robot < stamp_camera)
        {
            t_camera--;
            stamp_camera =
                camera_frontend_.get_timestamp_ms(t_camera);
        }

        return camera_frontend_.get_observation(t_camera);
    }


private:
    trifinger_object_tracking::ObjectTrackerFrontend object_tracker_frontend_;
    robot_interfaces::SensorFrontend<CameraObservation> camera_frontend_;
};
}  // namespace robot_fingers
