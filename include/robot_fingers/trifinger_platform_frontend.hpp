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
class TriFingerPlatformFrontend
{
public:
    // typedefs for easy access
    typedef robot_interfaces::TriFingerTypes::Action Action;
    typedef robot_interfaces::TriFingerTypes::Observation RobotObservation;
    typedef robot_interfaces::Status RobotStatus;
    typedef trifinger_cameras::TriCameraObservation CameraObservation;

    TriFingerPlatformFrontend(
        robot_interfaces::TriFingerTypes::BaseDataPtr robot_data,
        trifinger_object_tracking::ObjectTrackerData::Ptr object_tracker_data,
        std::shared_ptr<robot_interfaces::SensorData<CameraObservation>>
            camera_data);

    TriFingerPlatformFrontend();

    time_series::Index append_desired_action(const Action &desired_action);

    RobotObservation get_robot_observation(const time_series::Index &t) const;

    Action get_desired_action(const time_series::Index &t) const;

    Action get_applied_action(const time_series::Index &t) const;

    RobotStatus get_robot_status(const time_series::Index &t) const;

    time_series::Timestamp get_timestamp_ms(const time_series::Index &t) const;

    time_series::Index get_current_timeindex() const;

    void wait_until_timeindex(const time_series::Index &t) const;

    trifinger_object_tracking::ObjectPose get_object_pose(
        const time_series::Index t) const;

    CameraObservation get_camera_observation(const time_series::Index t) const;

private:
    robot_interfaces::TriFingerTypes::Frontend robot_frontend_;
    trifinger_object_tracking::ObjectTrackerFrontend object_tracker_frontend_;
    robot_interfaces::SensorFrontend<CameraObservation> camera_frontend_;

    template <typename FrontendType>
    time_series::Index find_matching_timeindex(const FrontendType &frontend,
                                               const time_series::Index t) const
    {
        // The given time index t refers to the robot data time series.  To
        // provide the correct observation from the other frontend for this time
        // step, find the highest time index t_o of the other frontend where
        //
        //     timestamp(t_o) <= timestamp(t)
        //
        // Note that this is not always the one that is closest w.r.t. to the
        // timestamp, i.e.
        //
        //     t_o != argmin(|timestamp(t_o) - timestamp(t)|)
        //
        // The latter would not be deterministic: the outcome could change when
        // called twice with the same `t` if a new "other" observation arrived
        // in between the calls.

        // TODO: The implementation below is very naive.
        // It simply does a linear search starting from the latest time index.
        // So worst case performance is O(n) where n is the number of "other"
        // observations over the period that is covered by the buffer of the
        // robot data.
        //
        // Options to speed this up:
        // - binary search (?)
        // - estimate time step size based on last observations
        // - store matched indices of last call
        //
        // Note, however, that `t` is very likely the latest time index in most
        // cases.  In this case the match for `t_o` will also be the latest
        // index of the corresponding time series.  In this case, the complexity
        // is O(1).  So even when implementing a more complex search algorithm,
        // the first candidate for `t_o` that is checked should always be the
        // latest one.

        time_series::Timestamp stamp_robot = get_timestamp_ms(t);

        time_series::Index t_other = frontend.get_current_timeindex();
        time_series::Timestamp stamp_other = frontend.get_timestamp_ms(t_other);

        while (stamp_robot < stamp_other)
        {
            t_other--;
            stamp_other = frontend.get_timestamp_ms(t_other);
        }

        return t_other;
    }
};
}  // namespace robot_fingers
