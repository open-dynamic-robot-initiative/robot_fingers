/**
 * \file
 * \brief Driver for a "Solo 8" robot.
 * \copyright Copyright (c) 2019, New York University and Max Planck
 *            Gesellschaft.
 */

#pragma once

#include <robot_fingers/n_joint_blmc_robot_driver.hpp>

namespace robot_fingers
{
/**
 * @brief Driver for Solo 8.
 *
 * Driver for 4 times double BLMC joint.
 */
class SoloEightDriver : public SimpleNJointBlmcRobotDriver<8, 4>
{
public:
    SoloEightDriver(const Config &config)
        : SoloEightDriver(create_motor_boards(config.can_ports), config)
    {
    }

private:
   SoloEightDriver(const MotorBoards &motor_boards, const Config &config)
        : SimpleNJointBlmcRobotDriver<8, 4>(motor_boards,
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
        motors[3] = std::make_shared<blmc_drivers::Motor>(motor_boards[1], 1);

        motors[4] = std::make_shared<blmc_drivers::Motor>(motor_boards[2], 0);
        motors[5] = std::make_shared<blmc_drivers::Motor>(motor_boards[2], 1);

        motors[6] = std::make_shared<blmc_drivers::Motor>(motor_boards[3], 0);
        motors[7] = std::make_shared<blmc_drivers::Motor>(motor_boards[3], 1);

        return motors;
    }
};

}  // namespace robot_fingers
