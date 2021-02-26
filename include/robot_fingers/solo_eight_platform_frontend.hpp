/**
 * @file
 * @brief Frontend for the Solo8 Platform
 * @copyright 2020, Max Planck Gesellschaft.  All rights reserved.
 * @license BSD 3-clause
 */
#pragma once

// TODO move to a separate package to not add unnecessary dependencies to
// robot_fingers?
#include <robot_interfaces/finger_types.hpp>
#include <robot_interfaces/sensors/sensor_frontend.hpp>

namespace robot_fingers
{
/**
 * @brief Frontend for the Solo8 Platform
 */
class SoloEightPlatformFrontend
{
public:
    // typedefs for easy access
    typedef robot_interfaces::SimpleNJointRobotTypes<8>::Action Action;
    typedef robot_interfaces::SimpleNJointRobotTypes<8>::Observation RobotObservation;
    typedef robot_interfaces::Status RobotStatus;

    /**
     * @brief Initialize with data instances for all internal frontends.
     *
     * @param robot_data  RobotData instance used by the robot frontend.
     */
    SoloEightPlatformFrontend(
        robot_interfaces::SimpleNJointRobotTypes<8>::BaseDataPtr robot_data);

    /**
     * @brief Initialize with default data instances.
     *
     * Creates for each internal frontend a corresponding mutli-process data
     * instance with the default shared memory ID for the corresponding data
     * type.
     */
    SoloEightPlatformFrontend();

    /**
     * @brief Append a desired robot action to the action queue.
     * @see robot_interfaces::SimpleNJointRobotTypes<8>::Frontend::append_desired_action
     *
     * @return The index of the time step at which this action is going to be
     *     executed.
     */
    time_series::Index append_desired_action(const Action &desired_action);

    /**
     * @brief Get robot observation of the time step t.
     * @see robot_interfaces::SimpleNJointRobotTypes<8>::Frontend::get_observation
     */
    RobotObservation get_robot_observation(const time_series::Index &t) const;

    /**
     * @brief Get desired action of time step t.
     * @see robot_interfaces::SimpleNJointRobotTypes<8>::Frontend::get_desired_action
     */
    Action get_desired_action(const time_series::Index &t) const;

    /**
     * @brief Get actually applied action of time step t.
     * @see robot_interfaces::SimpleNJointRobotTypes<8>::Frontend::get_applied_action
     */
    Action get_applied_action(const time_series::Index &t) const;

    /**
     * @brief Get robot status of time step t.
     * @see robot_interfaces::SimpleNJointRobotTypes<8>::Frontend::get_status
     */
    RobotStatus get_robot_status(const time_series::Index &t) const;

    /**
     * @brief Get timestamp (in milliseconds) of time step t.
     * @see robot_interfaces::SimpleNJointRobotTypes<8>::Frontend::get_timestamp_ms
     */
    time_series::Timestamp get_timestamp_ms(const time_series::Index &t) const;

    /**
     * @brief Get the current time index.
     * @see robot_interfaces::SimpleNJointRobotTypes<8>::Frontend::get_current_timeindex
     */
    time_series::Index get_current_timeindex() const;

    /**
     * @brief Wait until time step t.
     * @see robot_interfaces::SimpleNJointRobotTypes<8>::Frontend::wait_until_timeindex
     */
    void wait_until_timeindex(const time_series::Index &t) const;

private:
    robot_interfaces::SimpleNJointRobotTypes<8>::Frontend robot_frontend_;

};
}  // namespace robot_fingers
