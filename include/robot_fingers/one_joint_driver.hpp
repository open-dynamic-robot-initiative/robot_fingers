/**
 * \file
 * \brief Driver for a "one-joint" robot.
 * \copyright Copyright (c) 2019, New York University and Max Planck
 *            Gesellschaft.
 */

#pragma once

#include <blmc_robots/n_joint_blmc_robot_driver.hpp>

namespace robot_fingers
{
/**
 * @brief Driver for a single joint.
 *
 * Driver for a single BLMC joint.  Mostly intended for testing purposes.
 */
class OneJointDriver : public blmc_robots::NJointBlmcRobotDriver<1, 1>
{
public:
    OneJointDriver(const Config &config)
        : OneJointDriver(create_motor_boards(config.can_ports), config)
    {
    }

private:
    OneJointDriver(const MotorBoards &motor_boards, const Config &config)
        : blmc_robots::NJointBlmcRobotDriver<1, 1>(motor_boards,
                                      create_motors(motor_boards),
                                      {
                                          // MotorParameters
                                          .torque_constant_NmpA = 0.02,
                                          .gear_ratio = 9.0,
                                      },
                                      config)
    {
    }

    static Motors create_motors(const MotorBoards &motor_boards)
    {
        // set up motors
        Motors motors;
        motors[0] = std::make_shared<blmc_drivers::Motor>(motor_boards[0], 0);

        return motors;
    }
};

}  // namespace robot_fingers
