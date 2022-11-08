/**
 * @file
 * @brief Driver for the Finger Robots.
 * @copyright 2020, Max Planck Gesellschaft. All rights reserved.
 * @license BSD 3-clause
 */
#pragma once

#include <robot_fingers/n_joint_blmc_robot_driver.hpp>
#include <robot_interfaces/finger_types.hpp>

namespace robot_fingers
{
// alias for the base class, so all template arguments only need to be listed
// once
template <size_t N_FINGERS>
using _NFingerDriverBase =
    NJointBlmcRobotDriver<robot_interfaces::NFingerObservation<N_FINGERS>,
                          N_FINGERS * robot_interfaces::JOINTS_PER_FINGER,
                          N_FINGERS * robot_interfaces::BOARDS_PER_FINGER>;

/**
 * @brief Driver for the Finger robots.
 *
 * This is a generic driver for the CAN-based BLMC Finger Robots.  It works for
 * both a single finger and a set of multiple fingers.
 *
 * @tparam N_FINGERS  Number of fingers on the robot.
 */
template <size_t N_FINGERS>
class NFingerDriver : public _NFingerDriverBase<N_FINGERS>
{
public:
    typedef robot_interfaces::NFingerObservation<N_FINGERS> Observation;
    using typename _NFingerDriverBase<N_FINGERS>::Vector;

    // inherit the base constructor
    using _NFingerDriverBase<N_FINGERS>::_NFingerDriverBase;

    Observation get_latest_observation() override
    {
        Observation observation;

        observation.position = this->joint_modules_.get_measured_angles();
        observation.velocity = this->joint_modules_.get_measured_velocities();
        observation.torque = this->joint_modules_.get_measured_torques();

        for (size_t finger_idx = 0; finger_idx < N_FINGERS; finger_idx++)
        {
            // The force sensor is supposed to be connected to ADC A
            // (= analog_0) on the first board of each finger.
            const size_t board_idx =
                finger_idx * robot_interfaces::BOARDS_PER_FINGER;

            auto adc_a_history =
                this->motor_boards_[board_idx]->get_measurement(
                    blmc_drivers::MotorBoardInterface::MeasurementIndex::
                        analog_0);

            if (adc_a_history->length() == 0)
            {
                observation.tip_force[finger_idx] =
                    std::numeric_limits<double>::quiet_NaN();
            }
            else
            {
                observation.tip_force[finger_idx] =
                    adc_a_history->newest_element();
            }
        }

        return observation;
    }
};

}  // namespace robot_fingers
