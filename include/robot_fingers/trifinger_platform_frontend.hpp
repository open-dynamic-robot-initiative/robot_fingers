/**
 * @file
 * @brief Combined frontend for the TriFinger Platform
 * @copyright 2020, Max Planck Gesellschaft.  All rights reserved.
 * @license BSD 3-clause
 */
#pragma once

#include <robot_interfaces/finger_types.hpp>
#include <robot_interfaces/sensors/sensor_frontend.hpp>
#include <trifinger_cameras/tricamera_observation.hpp>
#include <trifinger_object_tracking/tricamera_object_observation.hpp>

namespace robot_fingers
{
/**
 * @brief Combined frontend for the TriFinger Platform
 *
 * This class combines the frontends for robot and cameras in one class using
 * unified time indices.
 *
 * Internally the different frontends all have their own time indices which are
 * unrelated to each other.  In this combined class, the time index used is the
 * one that belongs to the robot frontend.  When accessing observations of the
 * other frontends, it also takes this index t and internally matches it to the
 * time index t_o that was active in the other frontend at the time of t.
 *
 * @todo Methods to get timestamp from camera or object tracker?
 */
template <typename CameraObservation_t>
class T_TriFingerPlatformFrontend
{
public:
    // typedefs for easy access
    typedef robot_interfaces::TriFingerTypes::Action Action;
    typedef robot_interfaces::TriFingerTypes::Observation RobotObservation;
    typedef robot_interfaces::Status RobotStatus;
    // typedef trifinger_object_tracking::TriCameraObjectObservation
    //    CameraObservation;
    typedef CameraObservation_t CameraObservation;

    /**
     * @brief Initialize with data instances for all internal frontends.
     *
     * @param robot_data  RobotData instance used by the robot frontend.
     * @param object_tracker_data ObjectTrackerData instance used by the object
     *     tracker frontend.
     * @param camera_data SensorData instance, used by the camera frontend.
     */
    T_TriFingerPlatformFrontend(
        robot_interfaces::TriFingerTypes::BaseDataPtr robot_data,
        std::shared_ptr<robot_interfaces::SensorData<CameraObservation>>
            camera_data)
        : robot_frontend_(robot_data), camera_frontend_(camera_data)
    {
    }

    /**
     * @brief Initialize with default data instances.
     *
     * Creates for each internal frontend a corresponding mutli-process data
     * instance with the default shared memory ID for the corresponding data
     * type.
     */
    T_TriFingerPlatformFrontend()
        : robot_frontend_(std::make_shared<
                          robot_interfaces::TriFingerTypes::MultiProcessData>(
              "trifinger", false)),
          camera_frontend_(
              std::make_shared<
                  robot_interfaces::MultiProcessSensorData<CameraObservation>>(
                  "tricamera", false))

    {
    }

    /**
     * @brief Append a desired robot action to the action queue.
     * @see robot_interfaces::TriFingerTypes::Frontend::append_desired_action
     *
     * @return The index of the time step at which this action is going to be
     *     executed.
     */
    time_series::Index append_desired_action(const Action &desired_action)
    {
        return robot_frontend_.append_desired_action(desired_action);
    }

    /**
     * @brief Get robot observation of the time step t.
     * @see robot_interfaces::TriFingerTypes::Frontend::get_observation
     */
    RobotObservation get_robot_observation(const time_series::Index &t) const
    {
        return robot_frontend_.get_observation(t);
    }

    /**
     * @brief Get desired action of time step t.
     * @see robot_interfaces::TriFingerTypes::Frontend::get_desired_action
     */
    Action get_desired_action(const time_series::Index &t) const
    {
        return robot_frontend_.get_desired_action(t);
    }

    /**
     * @brief Get actually applied action of time step t.
     * @see robot_interfaces::TriFingerTypes::Frontend::get_applied_action
     */
    Action get_applied_action(const time_series::Index &t) const
    {
        return robot_frontend_.get_applied_action(t);
    }

    /**
     * @brief Get robot status of time step t.
     * @see robot_interfaces::TriFingerTypes::Frontend::get_status
     */
    RobotStatus get_robot_status(const time_series::Index &t) const
    {
        return robot_frontend_.get_status(t);
    }

    /**
     * @brief Get timestamp (in milliseconds) of time step t.
     * @see robot_interfaces::TriFingerTypes::Frontend::get_timestamp_ms
     */
    time_series::Timestamp get_timestamp_ms(const time_series::Index &t) const
    {
        return robot_frontend_.get_timestamp_ms(t);
    }

    /**
     * @brief Get the current time index.
     * @see robot_interfaces::TriFingerTypes::Frontend::get_current_timeindex
     */
    time_series::Index get_current_timeindex() const
    {
        return robot_frontend_.get_current_timeindex();
    }

    /**
     * @brief Wait until time step t.
     * @see robot_interfaces::TriFingerTypes::Frontend::wait_until_timeindex
     */
    void wait_until_timeindex(const time_series::Index &t) const
    {
        robot_frontend_.wait_until_timeindex(t);
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
        auto t_camera = find_matching_timeindex(camera_frontend_, t);
        return camera_frontend_.get_observation(t_camera);
    }

private:
    robot_interfaces::TriFingerTypes::Frontend robot_frontend_;
    robot_interfaces::SensorFrontend<CameraObservation> camera_frontend_;

    /**
     * @brief Find time index of frontend that matches with the given robot time
     *        index.
     *
     * The given time index t_robot refers to the robot data time series.  To
     * provide the correct observation from the other frontend for this time
     * step, find the highest time index t_other of the other frontend where
     *
     *      timestamp(t_other) <= timestamp(t_robot)
     *
     * Note that this is not always the one that is closest w.r.t. to the
     * timestamp, i.e.
     *
     *      t_other != argmin(|timestamp(t_other) - timestamp(t_robot)|)
     *
     * The latter would not be deterministic: the outcome could change when
     * called twice with the same `t_robot` if a new "other" observation
     * arrived in between the calls.
     *
     * @todo The implementation below is very naive.
     *       It simply does a linear search starting from the latest time index.
     *       So worst case performance is O(n) where n is the number of "other"
     *       observations over the period that is covered by the buffer of the
     *       robot data.
     *
     *       Options to speed this up:
     *        - binary search (?)
     *        - estimate time step size based on last observations
     *        - store matched indices of last call
     *
     *       Note, however, that `t_robot` is very likely the latest time index
     *       in most cases.  In this case the match for `t_other` will also be
     *       the latest index of the corresponding time series.  In this case,
     *       the complexity is O(1).  So even when implementing a more complex
     *       search algorithm, the first candidate for `t_other` that is checked
     *       should always be the latest one.
     *
     * @tparam FrontendType Type of the frontend.  This is templated so that the
     *     same implementation can be used for both camera and object tracker
     *     frontend.
     * @param other_frontend The frontend for which a matching time index needs
     *     to be found.
     * @param t_robot Time index of the robot frontend.
     *
     * @return Time index for other_frontend which is/was active at the time of
     *     t_robot.
     */
    template <typename FrontendType>
    time_series::Index find_matching_timeindex(
        const FrontendType &other_frontend,
        const time_series::Index t_robot) const
    {
        time_series::Timestamp stamp_robot = get_timestamp_ms(t_robot);

        time_series::Index t_other = other_frontend.get_current_timeindex();
        time_series::Timestamp stamp_other =
            other_frontend.get_timestamp_ms(t_other);

        while (stamp_robot < stamp_other)
        {
            t_other--;
            stamp_other = other_frontend.get_timestamp_ms(t_other);
        }

        return t_other;
    }
};

// typedefs for easier use
typedef T_TriFingerPlatformFrontend<
    trifinger_object_tracking::TriCameraObjectObservation>
    TriFingerPlatformWithObjectFrontend;
typedef T_TriFingerPlatformFrontend<trifinger_cameras::TriCameraObservation>
    TriFingerPlatformFrontend;
}  // namespace robot_fingers
