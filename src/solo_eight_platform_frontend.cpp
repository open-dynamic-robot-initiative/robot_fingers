/**
 * @file
 * @brief Frontend for the Solo8 Platform
 * @copyright 2020, Max Planck Gesellschaft.  All rights reserved.
 * @license BSD 3-clause
 */
#include <robot_fingers/solo_eight_platform_frontend.hpp>

namespace robot_fingers
{
SoloEightPlatformFrontend::SoloEightPlatformFrontend(
    robot_interfaces::SimpleNJointRobotTypes<8>::BaseDataPtr robot_data)
    : robot_frontend_(robot_data)
{
}

SoloEightPlatformFrontend::SoloEightPlatformFrontend()
    : robot_frontend_(
          std::make_shared<robot_interfaces::SimpleNJointRobotTypes<8>::MultiProcessData>(
              "solo8", false))
{
}

time_series::Index SoloEightPlatformFrontend::append_desired_action(
    const Action &desired_action)
{
    return robot_frontend_.append_desired_action(desired_action);
}

SoloEightPlatformFrontend::RobotObservation
SoloEightPlatformFrontend::get_robot_observation(
    const time_series::Index &t) const
{
    return robot_frontend_.get_observation(t);
}

SoloEightPlatformFrontend::Action SoloEightPlatformFrontend::get_desired_action(
    const time_series::Index &t) const
{
    return robot_frontend_.get_desired_action(t);
}

SoloEightPlatformFrontend::Action SoloEightPlatformFrontend::get_applied_action(
    const time_series::Index &t) const
{
    return robot_frontend_.get_applied_action(t);
}

SoloEightPlatformFrontend::RobotStatus
SoloEightPlatformFrontend::get_robot_status(const time_series::Index &t) const
{
    return robot_frontend_.get_status(t);
}

time_series::Timestamp SoloEightPlatformFrontend::get_timestamp_ms(
    const time_series::Index &t) const
{
    return robot_frontend_.get_timestamp_ms(t);
}

time_series::Index SoloEightPlatformFrontend::get_current_timeindex() const
{
    return robot_frontend_.get_current_timeindex();
}

void SoloEightPlatformFrontend::wait_until_timeindex(
    const time_series::Index &t) const
{
    robot_frontend_.wait_until_timeindex(t);
}

}  // namespace robot_fingers
