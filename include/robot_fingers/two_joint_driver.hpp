/**
 * \file
 * \brief Driver for a "two-joint" robot.
 * \copyright Copyright (c) 2019, New York University and Max Planck
 *            Gesellschaft.
 */

#pragma once

#include <robot_fingers/n_joint_blmc_robot_driver.hpp>

namespace robot_fingers
{
/**
 * @brief Driver for two joints.
 *
 * Driver for a double BLMC joint.  Mostly intended for testing purposes.
 */
class TwoJointDriver : public SimpleNJointBlmcRobotDriver<2, 1>
{
public:
    TwoJointDriver(const Config &config)
        : TwoJointDriver(create_motor_boards(config.can_ports), config)
    {
    }

private:
    TwoJointDriver(const MotorBoards &motor_boards, const Config &config)
        : SimpleNJointBlmcRobotDriver<2, 1>(
              motor_boards,
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
        motors[1] = std::make_shared<blmc_drivers::Motor>(motor_boards[0], 1);

        return motors;
    }
};

}  // namespace robot_fingers
