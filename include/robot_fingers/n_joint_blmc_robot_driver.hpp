/**
 * \file
 * \brief Base driver for a generic n-joint BLMC robot.
 * \copyright Copyright (c) 2019, New York University and Max Planck
 *            Gesellschaft.
 */

#pragma once

#include <array>
#include <cmath>
#include <iterator>
#include <string>

#include <yaml-cpp/yaml.h>
#include <Eigen/Eigen>

#include <yaml_cpp_catkin/yaml_eigen.h>
#include <mpi_cpp_tools/math.hpp>
#include <robot_interfaces/n_joint_robot_types.hpp>
#include <robot_interfaces/robot_driver.hpp>

#include <blmc_robots/blmc_joint_module.hpp>
#include <blmc_robots/common_header.hpp>

namespace blmc_robots
{
/**
 * @brief Parameters related to the motor.
 */
struct MotorParameters
{
    //! @brief Torque constant K_t of the motor [Nm/A].
    double torque_constant_NmpA;

    /**
     * @brief Gear ratio between motor and joint.
     *
     * For a `n:1` ratio (i.e. one joint revolution requires n motor
     * revolutions) set this value to `n`.
     */
    double gear_ratio;
};

/**
 * @brief Parameters for the joint calibration (homing and go to initial pose).
 */
struct CalibrationParameters
{
    //! @brief Torque that is used to find the end stop.
    double endstop_search_torque_Nm;
    //! @brief Tolerance for reaching the starting position.
    double position_tolerance_rad;
    //! @brief Timeout for reaching the starting position.
    double move_timeout;
};

/**
 * @brief Base class for simple n-joint BLMC robots.
 *
 * This is a generic base class to easily implement drivers for simple BLMC
 * robots that consist of N_JOINTS joints.
 *
 * @tparam N_JOINTS Number of joints.
 * @tparam N_MOTOR_BOARDS Number of motor control boards that are used.
 */
template <size_t N_JOINTS, size_t N_MOTOR_BOARDS>
class NJointBlmcRobotDriver
    : public robot_interfaces::RobotDriver<
          typename robot_interfaces::NJointRobotTypes<N_JOINTS>::Action,
          typename robot_interfaces::NJointRobotTypes<N_JOINTS>::Observation>
{
public:
    typedef typename robot_interfaces::NJointRobotTypes<N_JOINTS> Types;

    typedef typename Types::Action Action;
    typedef typename Types::Observation Observation;
    typedef typename Types::Vector Vector;
    typedef std::array<std::shared_ptr<blmc_drivers::MotorInterface>, N_JOINTS>
        Motors;
    typedef std::array<std::shared_ptr<blmc_drivers::CanBusMotorBoard>,
                       N_MOTOR_BOARDS>
        MotorBoards;

    /**
     * @brief Configuration of the robot that can be changed by the user.
     */
    struct Config
    {
        typedef std::array<std::string, N_MOTOR_BOARDS> CanPortArray;

        // All parameters should have default values that should not result in
        // dangerous behaviour in case someone forgets to specify them.

        /**
         * @brief List of CAN port names used by the robot.
         *
         * For each motor control board used by the robot, this specifies the
         * CAN port through which it is connected.
         *
         * Example: `{"can0", "can1"}`
         */
        CanPortArray can_ports;

        //! @brief Maximum current that can be sent to the motor [A].
        double max_current_A = 0.0;

        /**
         * @brief Whether the joints have physical end stops or not.
         *
         * This is for example relevant for homing, where (in case this value
         * is set to true) all joints move until they hit the end stop to
         * determine their absolute position.
         *
         * Note that not having end stops does not mean that the joint can
         * rotate freely in general.
         */
        bool has_endstop = false;

        //! @brief Parameters related to calibration.
        CalibrationParameters calibration = {
            .endstop_search_torque_Nm = 0.0,
            .position_tolerance_rad = 0.0,
            .move_timeout = 0,
        };

        //! \brief D-gain to dampen velocity.  Set to zero to disable damping.
        // set some rather high damping by default
        Vector safety_kd = Vector::Constant(0.1);

        //! @brief Default control gains for the position PD controller.
        struct
        {
            Vector kp = Vector::Zero();
            Vector kd = Vector::Zero();
        } position_control_gains;

        //! \brief Offset between home position and zero.
        Vector home_offset_rad = Vector::Zero();

        /**
         * @brief Initial position to which the robot moves after
         *        initialization.
         */
        Vector initial_position_rad = Vector::Zero();

        /**
         * @brief Print the given configuration in a human-readable way.
         */
        void print() const
        {
            std::cout << "Configuration:\n"
                      << "\t can_ports:";
            for (const auto &port : can_ports)
            {
                std::cout << " " << port;
            }
            std::cout << "\n"
                      << "\t max_current_A: " << max_current_A << "\n"
                      << "\t has_endstop: " << has_endstop << "\n"
                      << "\t calibration: "
                      << "\n"
                      << "\t\t endstop_search_torque_Nm: "
                      << calibration.endstop_search_torque_Nm << "\n"
                      << "\t\t position_tolerance_rad: "
                      << calibration.position_tolerance_rad << "\n"
                      << "\t\t move_timeout: " << calibration.move_timeout
                      << "\n"
                      << "\t safety_kd: " << safety_kd.transpose() << "\n"
                      << "\t position_control_gains: "
                      << "\n"
                      << "\t\t kp: " << position_control_gains.kp.transpose()
                      << "\n"
                      << "\t\t kd: " << position_control_gains.kd.transpose()
                      << "\n"
                      << "\t home_offset_rad: " << home_offset_rad.transpose()
                      << "\n"
                      << "\t initial_position_rad: "
                      << initial_position_rad.transpose() << "\n"
                      << std::endl;
        }

        /**
         * @brief Load driver configuration from file.
         *
         * Load the configuration from the specified YAML file.  The file is
         * expected to have the same structure/key naming as the Config struct.
         * If a value can not be read from the file, the application exists
         * with an error message.
         *
         * @param config_file_name  Path/name of the configuration YAML file.
         *
         * @return Configuration
         */
        static Config load_config(const std::string &config_file_name)
        {
            Config config;
            YAML::Node user_config;

            try
            {
                user_config = YAML::LoadFile(config_file_name);
            }
            catch (...)
            {
                std::cout << "FATAL: Failed to load configuration from '"
                          << config_file_name << "'." << std::endl;
                std::exit(1);
            }

            // replace values from the default config with the ones given in the
            // users config file

            // TODO: for some reason direct conversion is not working (despite
            // yaml-cpp implementing a generic conversion for std::array)
            // set_config_value<CanPortArray>(user_config, "can_ports",
            // &config.can_ports);
            try
            {
                for (size_t i = 0; i < config.can_ports.size(); i++)
                {
                    config.can_ports[i] =
                        user_config["can_ports"][i].as<std::string>();
                }
            }
            catch (...)
            {
                std::cerr << "FATAL: Failed to load parameter 'can_ports' from "
                             "configuration file"
                          << std::endl;
                std::exit(1);
            }

            set_config_value(
                user_config, "max_current_A", &config.max_current_A);
            set_config_value(user_config, "has_endstop", &config.has_endstop);

            if (user_config["calibration"])
            {
                YAML::Node calib = user_config["calibration"];

                set_config_value(calib,
                                 "endstop_search_torque_Nm",
                                 &config.calibration.endstop_search_torque_Nm);
                set_config_value(calib,
                                 "position_tolerance_rad",
                                 &config.calibration.position_tolerance_rad);
                set_config_value(
                    calib, "move_timeout", &config.calibration.move_timeout);
            }

            set_config_value(user_config, "safety_kd", &config.safety_kd);

            if (user_config["position_control_gains"])
            {
                YAML::Node pos_ctrl = user_config["position_control_gains"];

                set_config_value(
                    pos_ctrl, "kp", &config.position_control_gains.kp);
                set_config_value(
                    pos_ctrl, "kd", &config.position_control_gains.kd);
            }

            set_config_value(
                user_config, "home_offset_rad", &config.home_offset_rad);
            set_config_value(user_config,
                             "initial_position_rad",
                             &config.initial_position_rad);

            return config;
        }

    private:
        /**
         * \brief Set value from user configuration to var if specified.
         *
         * Checks if a field `name` exists in `user_config`.  If yes, its value
         * is written to `var`, otherwise `var` is unchanged.
         *
         * \param[in] user_config  YAML node containing the user configuration.
         * \param[in] name  Name of the configuration entry.
         * \param[out] var  Variable to which configuration is written.  Value
         * is unchanged if the specified field name does not exist in
         * user_config, i.e.  it can be initialized with a default value.
         */
        template <typename T>
        static void set_config_value(const YAML::Node &user_config,
                                     const std::string &name,
                                     T *var)
        {
            try
            {
                *var = user_config[name].as<T>();
            }
            catch (const YAML::Exception &e)
            {
                std::cerr << "FATAL: Failed to load parameter '" << name
                          << "' from configuration file" << std::endl;
                std::exit(1);
            };
        }
    };

    /**
     * @brief True if the joints have mechanical end stops, false if not.
     *
     * If set to true, it is assumed that all joints of the robot have
     * end stops that mechanically prevent them from moving out of the valid
     * range.
     *
     * If present, the end stops are used for a fully automated homing procedure
     * in which the joints first move until they hit the end stop before
     * starting the encoder index search.  This way it is ensured that the
     * correct index is used for homing without any need for manual set up.
     */
    const bool has_endstop_;

    NJointBlmcRobotDriver(const MotorBoards &motor_boards,
                          const Motors &motors,
                          const MotorParameters &motor_parameters,
                          const Config &config)
        : robot_interfaces::RobotDriver<Action, Observation>(),
          has_endstop_(config.has_endstop),
          joint_modules_(motors,
                         motor_parameters.torque_constant_NmpA * Vector::Ones(),
                         motor_parameters.gear_ratio * Vector::Ones(),
                         Vector::Zero(),
                         config.max_current_A * Vector::Ones()),
          motor_boards_(motor_boards),
          motor_parameters_(motor_parameters),
          max_torque_Nm_(config.max_current_A *
                         motor_parameters.torque_constant_NmpA *
                         motor_parameters.gear_ratio),
          config_(config)
    {
        pause_motors();
    }

    static MotorBoards create_motor_boards(
        const std::array<std::string, N_MOTOR_BOARDS> &can_ports)
    {
        // setup can buses -----------------------------------------------------
        std::array<std::shared_ptr<blmc_drivers::CanBus>, N_MOTOR_BOARDS>
            can_buses;
        for (size_t i = 0; i < can_buses.size(); i++)
        {
            can_buses[i] = std::make_shared<blmc_drivers::CanBus>(can_ports[i]);
        }

        // set up motor boards -------------------------------------------------
        MotorBoards motor_boards;
        for (size_t i = 0; i < motor_boards.size(); i++)
        {
            motor_boards[i] = std::make_shared<blmc_drivers::CanBusMotorBoard>(
                can_buses[i], 1000, 10);
            /// \TODO: reduce the timeout further!!
        }

        for (size_t i = 0; i < motor_boards.size(); i++)
        {
            motor_boards[i]->wait_until_ready();
        }

        return motor_boards;
    }

    void pause_motors()
    {
        for (size_t i = 0; i < motor_boards_.size(); i++)
        {
            motor_boards_[i]->pause_motors();
        }
    }

    Vector get_measured_index_angles() const
    {
        return joint_modules_.get_measured_index_angles();
    }

public:
    Observation get_latest_observation() override
    {
        Observation observation;
        observation.position = joint_modules_.get_measured_angles();
        observation.velocity = joint_modules_.get_measured_velocities();
        observation.torque = joint_modules_.get_measured_torques();
        return observation;
    }

protected:
    Action apply_action(const Action &desired_action) override
    {
        if (!is_initialized_)
        {
            throw std::runtime_error(
                "Robot needs to be initialized before applying actions.  Run "
                "the `initialize()` method.");
        }

        return apply_action_uninitialized(desired_action);
    }

    Action apply_action_uninitialized(const Action &desired_action)
    {
        double start_time_sec = real_time_tools::Timer::get_current_time_sec();

        Observation observation = get_latest_observation();

        Action applied_action;

        applied_action.torque = desired_action.torque;
        applied_action.position = desired_action.position;

        // Position controller
        // -------------------
        // TODO: add position limits
        //
        // NOTE: in the following lines, the Eigen function `unaryExpr` is
        // used to determine which elements of the vectors are NaN.  This is
        // because `isNaN()` was only added in Eigen 3.3 but we have to be
        // compatible with 3.2.
        // The std::isnan call needs to be wrapped in a lambda because it is
        // overloaded and unaryExpr cannot resolve which version to use when
        // passed directly.

        // Run the position controller only if a target position is set for at
        // least one joint.
        if (!applied_action.position
                 .unaryExpr([](double x) { return std::isnan(x); })
                 .all())
        {
            // Replace NaN-values with default gains
            applied_action.position_kp =
                applied_action.position_kp
                    .unaryExpr([](double x) { return std::isnan(x); })
                    .select(config_.position_control_gains.kp,
                            desired_action.position_kp);
            applied_action.position_kd =
                applied_action.position_kd
                    .unaryExpr([](double x) { return std::isnan(x); })
                    .select(config_.position_control_gains.kd,
                            desired_action.position_kd);

            Vector position_error =
                applied_action.position - observation.position;

            // simple PD controller
            Vector position_control_torque =
                applied_action.position_kp.cwiseProduct(position_error) +
                applied_action.position_kd.cwiseProduct(observation.velocity);

            // position_control_torque contains NaN for joints where target
            // position is set to NaN!  Filter those out and set the torque to
            // zero instead.
            position_control_torque =
                position_control_torque
                    .unaryExpr([](double x) { return std::isnan(x); })
                    .select(0, position_control_torque);

            // Add result of position controller to the torque command
            applied_action.torque += position_control_torque;
        }

        // Safety Checks
        // -------------
        // limit to configured maximum torque
        applied_action.torque =
            mct::clamp(applied_action.torque, -max_torque_Nm_, max_torque_Nm_);
        // velocity damping to prevent too fast movements
        applied_action.torque -=
            config_.safety_kd.cwiseProduct(observation.velocity);
        // after applying checks, make sure we are still below the max. torque
        applied_action.torque =
            mct::clamp(applied_action.torque, -max_torque_Nm_, max_torque_Nm_);

        joint_modules_.set_torques(applied_action.torque);
        joint_modules_.send_torques();

        real_time_tools::Timer::sleep_until_sec(start_time_sec + 0.001);

        return applied_action;
    }

    void shutdown() override
    {
        pause_motors();
    }

    /**
     * @brief Homing on negative end stop and encoder index.
     *
     * Procedure for finding an absolute zero position (or "home" position) when
     * using relative encoders.
     *
     * If the robot has end stops (according to configuration), all joints first
     * move in negative direction until they hit the end stop.
     * Then an encoder index search is started where each joint moves slowly in
     * positive direction until the next encoder index.  The position of this
     * encoder index is the "home position".
     *
     * By default, the zero position is the same as the home position.  The
     * optional argument home_offset_rad provides a means to move the zero
     * position
     * relative to the home position.  The zero position is computed as
     *
     *     zero position = encoder index position + home offset
     *
     *
     * @param endstop_search_torque_Nm Torque that is used to move the joints
     *     while searching the end stop.
     * @param home_offset_rad Offset between the home position and the desired
     *     zero position.
     */
    bool home_on_index_after_negative_end_stop(
        double endstop_search_torque_Nm,
        Vector home_offset_rad = Vector::Zero())
    {
        //! Distance after which encoder index search is aborted.
        //! Computed based on gear ratio to be 1.5 motor revolutions.
        const double SEARCH_DISTANCE_LIMIT_RAD =
            (1.5 / motor_parameters_.gear_ratio) * 2 * M_PI;

        rt_printf("Start homing.\n");
        if (has_endstop_)
        {
            //! Min. number of steps when moving to the end stop.
            constexpr uint32_t MIN_STEPS_MOVE_TO_END_STOP = 1000;
            //! Size of the window when computing average velocity.
            constexpr uint32_t SIZE_VELOCITY_WINDOW = 100;
            //! Velocity limit at which the joints are considered to be stopped
            constexpr double STOP_VELOCITY = 0.005;

            static_assert(MIN_STEPS_MOVE_TO_END_STOP > SIZE_VELOCITY_WINDOW,
                          "MIN_STEPS_MOVE_TO_END_STOP has to be bigger than"
                          " SIZE_VELOCITY_WINDOW to ensure correct computation"
                          " of average velocity.");

            // Move until velocity drops to almost zero (= joints hit the end
            // stops) but at least for MIN_STEPS_MOVE_TO_END_STOP time steps.
            // TODO: add timeout to this loop?
            std::vector<Vector> running_velocities(SIZE_VELOCITY_WINDOW);
            Vector summed_velocities = Vector::Zero();
            uint32_t step_count = 0;
            while (step_count < MIN_STEPS_MOVE_TO_END_STOP ||
                   (summed_velocities.maxCoeff() / SIZE_VELOCITY_WINDOW >
                    STOP_VELOCITY))
            {
                Vector torques = Vector::Constant(-endstop_search_torque_Nm);
                apply_action_uninitialized(torques);
                Vector abs_velocities =
                    get_latest_observation().velocity.cwiseAbs();

                uint32_t running_index = step_count % SIZE_VELOCITY_WINDOW;
                if (step_count >= SIZE_VELOCITY_WINDOW)
                {
                    summed_velocities -= running_velocities[running_index];
                }
                running_velocities[running_index] = abs_velocities;
                summed_velocities += abs_velocities;
                step_count++;

#ifdef VERBOSE
                Eigen::IOFormat commainitfmt(
                    4, Eigen::DontAlignCols, " ", " ", "", "", "", "");
                std::cout << ((summed_velocities / SIZE_VELOCITY_WINDOW)
                                  .array() > STOP_VELOCITY)
                                 .format(commainitfmt)
                          << std::endl;
#endif
            }
            rt_printf("Reached end stop.\n");
        }

        // Home on encoder index
        HomingReturnCode homing_status = joint_modules_.execute_homing(
            SEARCH_DISTANCE_LIMIT_RAD, home_offset_rad);

        rt_printf("Finished homing.\n");

        return homing_status == HomingReturnCode::SUCCEEDED;
    }

    /**
     * @brief Move to given goal position using PD control.
     *
     * @param goal_pos Angular goal position for each joint.
     * @param tolerance Allowed position error for reaching the goal.  This is
     *     checked per joint, that is the maximal possible error is +/-tolerance
     *     on each joint.
     * @param timeout_cycles Timeout.  If exceeded before goal is reached, the
     *     procedure is aborted. Unit: Number of control loop cycles.
     * @return True if goal position is reached, false if timeout is exceeded.
     */
    bool move_to_position(const Vector &goal_pos,
                          const double tolerance,
                          const uint32_t timeout_cycles)
    {
        bool reached_goal = false;
        uint32_t cycle_count = 0;

        while (!reached_goal && cycle_count < timeout_cycles)
        {
            apply_action(Action::Position(goal_pos));

            const Vector position_error =
                goal_pos - get_latest_observation().position;
            const Vector velocity = get_latest_observation().velocity;

            // Check if the goal is reached (position error below tolerance and
            // velocity close to zero).
            constexpr double ZERO_VELOCITY = 1e-4;
            reached_goal = ((position_error.array().abs() < tolerance).all() &&
                            (velocity.array().abs() < ZERO_VELOCITY).all());

            cycle_count++;
        }

        return reached_goal;
    }

    /**
     * @brief Find home position of all joints and move to start position.
     *
     * Homes all joints using home_on_index_after_negative_end_stop.  When
     * finished, move the joint to the starting position (defined in
     * `config_.initial_position_rad`).
     *
     */
    void initialize() override
    {
        joint_modules_.set_position_control_gains(
            config_.position_control_gains.kp,
            config_.position_control_gains.kd);

        is_initialized_ = home_on_index_after_negative_end_stop(
            config_.calibration.endstop_search_torque_Nm,
            config_.home_offset_rad);

        if (is_initialized_)
        {
            bool reached_goal =
                move_to_position(config_.initial_position_rad,
                                 config_.calibration.position_tolerance_rad,
                                 config_.calibration.move_timeout);
            if (!reached_goal)
            {
                rt_printf("Failed to reach goal, timeout exceeded.\n");
            }
        }

        pause_motors();
    }

protected:
    BlmcJointModules<N_JOINTS> joint_modules_;
    MotorBoards motor_boards_;

    //! \brief Fixed motor parameters (assuming all joints use same setup).
    MotorParameters motor_parameters_;

    //! \brief Maximum torque allowed on each joint.
    double max_torque_Nm_;

    /**
     * \brief User-defined configuration of the driver
     *
     * Contains all configuration values that can be modified by the user (via
     * the configuration file).
     */
    Config config_;

    bool is_initialized_ = false;

public:
    Vector get_max_torques() const
    {
        return max_torque_Nm_ * Vector::Ones();
    }
};

template <typename Driver>
typename Driver::Types::BackendPtr create_backend(
    typename Driver::Types::DataPtr robot_data,
    const std::string &config_file_path)
{
    constexpr double MAX_ACTION_DURATION_S = 0.003;
    constexpr double MAX_INTER_ACTION_DURATION_S = 0.005;

    std::cout << "Load robot driver configuration from file '"
              << config_file_path << "'" << std::endl;
    auto config = Driver::Config::load_config(config_file_path);
    config.print();

    auto robot = std::make_shared<Driver>(config);

    auto backend = std::make_shared<typename Driver::Types::Backend>(
        robot, robot_data, MAX_ACTION_DURATION_S, MAX_INTER_ACTION_DURATION_S);
    backend->set_max_action_repetitions(std::numeric_limits<uint32_t>::max());

    return backend;
}

}  // namespace blmc_robots
