/**
 * @file
 * @brief Driver for the Finger Robots.
 * @copyright 2020, Max Planck Gesellschaft. All rights reserved.
 * @license BSD 3-clause
 */
#pragma once

#include <blmc_robots/n_joint_blmc_robot_driver.hpp>
#include <robot_interfaces/finger_types.hpp>

namespace robot_fingers
{
/**
 * @brief Driver for the Finger robots.
 *
 * This is a generic driver for the CAN-based BLMC Finger Robots.  It works for
 * both a single finger and a set of multiple fingers.
 *
 * @tparam N_FINGERS  Number of fingers on the robot.
 */
template <size_t N_FINGERS>
class FingerDriver : public blmc_robots::NJointBlmcRobotDriver<
                         robot_interfaces::FingerObservation<N_FINGERS>,
                         N_FINGERS * robot_interfaces::JOINTS_PER_FINGER,
                         N_FINGERS * robot_interfaces::BOARDS_PER_FINGER>
{
public:
    typedef robot_interfaces::FingerObservation<N_FINGERS> Observation;

    using blmc_robots::NJointBlmcRobotDriver<
        robot_interfaces::FingerObservation<N_FINGERS>,
        N_FINGERS * robot_interfaces::JOINTS_PER_FINGER,
        N_FINGERS * robot_interfaces::BOARDS_PER_FINGER>::NJointBlmcRobotDriver;

    Observation get_latest_observation() override
    {
        Observation observation;

        observation.position = this->joint_modules_.get_measured_angles();
        observation.velocity = this->joint_modules_.get_measured_velocities();
        observation.torque = this->joint_modules_.get_measured_torques();

        for (size_t finger_idx = 0; finger_idx < N_FINGERS; finger_idx++)
        {
            // The force sensor is supposed to be connected to ADC A on the
            // first board of each finger.
            const size_t board_idx =
                finger_idx * robot_interfaces::BOARDS_PER_FINGER;

            auto adc_a_history =
                this->motor_boards_[board_idx]->get_measurement(
                    blmc_drivers::MotorBoardInterface::MeasurementIndex::
                        analog_1);

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
