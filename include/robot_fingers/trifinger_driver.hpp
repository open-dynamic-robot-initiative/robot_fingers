/**
 * \file
 * \brief The hardware wrapper of the real TriFinger robot.
 * \copyright Copyright (c) 2019, New York University and Max Planck
 *            Gesellschaft.
 */

#pragma once

#include <blmc_robots/n_joint_blmc_robot_driver.hpp>
#include <robot_interfaces/trifinger_types.hpp>

namespace robot_fingers
{
class TriFingerDriver : public blmc_robots::NJointBlmcRobotDriver<9, 6>
{
public:
    TriFingerDriver(const Config &config)
        : TriFingerDriver(create_motor_boards(config.can_ports), config)
    {
    }

private:
    TriFingerDriver(const MotorBoards &motor_boards, const Config &config)
        : blmc_robots::NJointBlmcRobotDriver<9, 6>(
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
        Motors motors;

        // there are three fingers
        // each finger has three motors and two boards
        motors[0] = std::make_shared<blmc_drivers::Motor>(motor_boards[0], 0);
        motors[1] = std::make_shared<blmc_drivers::Motor>(motor_boards[0], 1);
        motors[2] = std::make_shared<blmc_drivers::Motor>(motor_boards[1], 0);

        motors[3] = std::make_shared<blmc_drivers::Motor>(motor_boards[2], 0);
        motors[4] = std::make_shared<blmc_drivers::Motor>(motor_boards[2], 1);
        motors[5] = std::make_shared<blmc_drivers::Motor>(motor_boards[3], 0);

        motors[6] = std::make_shared<blmc_drivers::Motor>(motor_boards[4], 0);
        motors[7] = std::make_shared<blmc_drivers::Motor>(motor_boards[4], 1);
        motors[8] = std::make_shared<blmc_drivers::Motor>(motor_boards[5], 0);

        return motors;
    }
};

}  // namespace robot_fingers
