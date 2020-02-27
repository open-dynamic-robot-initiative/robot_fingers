/**
 * \file
 * \brief The hardware wrapper of the real Finger robot.
 * \author Manuel Wuthrich
 * \date 2018
 * \copyright Copyright (c) 2019, New York University and Max Planck
 *            Gesellschaft.
 */

#pragma once

#include <robot_interfaces/finger_types.hpp>
#include <blmc_robots/n_joint_blmc_robot_driver.hpp>

namespace blmc_robots
{
class RealFingerDriver : public NJointBlmcRobotDriver<3, 2>
{
public:
    RealFingerDriver(const Config &config)
        : RealFingerDriver(create_motor_boards(config.can_ports), config)
    {
    }

private:
    RealFingerDriver(const MotorBoards &motor_boards, const Config &config)
        : NJointBlmcRobotDriver<3, 2>(motor_boards,
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
        motors[2] = std::make_shared<blmc_drivers::Motor>(motor_boards[1], 0);

        return motors;
    }
};

}  // namespace blmc_robots
