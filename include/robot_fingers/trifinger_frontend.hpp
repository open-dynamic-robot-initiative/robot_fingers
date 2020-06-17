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
// FIXME rename file
// TODO move implementation to cpp file
// NOTE: Inheriting from Frontend makes implementation easier but means that we
// cannot rename get_observation
class TriFingerPlattform : public robot_interfaces::TriFingerTypes::Frontend
{
public:
    TriFingerPlattform(
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

    trifinger_object_tracking::ObjectPose get_object_pose(const TimeIndex t)
    {
        // FIXME match to correct t
        return object_tracker_frontend_.get_current_pose();
    }

private:
    trifinger_object_tracking::ObjectTrackerFrontend object_tracker_frontend_;
};
}  // namespace robot_fingers
