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
#include <trifinger_object_tracking/object_tracker_data.hpp>

namespace robot_fingers
{
// TODO move implementation to cpp file
// NOTE: Inheriting from Frontend makes implementation easier but means that we
// cannot rename get_observation
class TriFingerPlatform : public robot_interfaces::TriFingerTypes::Frontend
{
public:
    TriFingerPlatform(
        robot_interfaces::TriFingerTypes::Data::Ptr robot_data,
        trifinger_object_tracking::ObjectTrackerData::Ptr object_tracker_data)
        : robot_interfaces::TriFingerTypes::Frontend(robot_data),
          object_tracker_frontend_(object_tracker_data)
    {
    }

    // alias
    robot_interfaces::TriFingerTypes::Observation get_robot_observation(
        const TimeIndex t)
    {
        return get_observation(t);
    }

    trifinger_object_tracking::ObjectPose get_object_pose(
        const time_series::Index t)
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

        time_series::Index t_tracker;
        auto stamp_robot = get_timestamp_ms(t);

        do
        {
            t_tracker = object_tracker_frontend_.get_current_timeindex();
            auto stamp_tracker =
                object_tracker_frontend_.get_timestamp_ms(t_tracker);
        } while (stamp_robot < stamp_tracker);

        return object_tracker_frontend_.get_pose(t_tracker);
    }

private:
    trifinger_object_tracking::ObjectTrackerFrontend object_tracker_frontend_;
};
}  // namespace robot_fingers
