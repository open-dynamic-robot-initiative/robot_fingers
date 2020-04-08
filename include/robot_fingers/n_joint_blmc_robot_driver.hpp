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
#include <robot_interfaces/monitored_robot_driver.hpp>
#include <robot_interfaces/n_joint_robot_functions.hpp>
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

    struct Config;  // actual declaration see below

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
        const std::array<std::string, N_MOTOR_BOARDS> &can_ports);

    Vector get_max_torques() const
    {
        return max_torque_Nm_ * Vector::Ones();
    }

    void pause_motors();

    Vector get_measured_index_angles() const;

    /**
     * @brief Find home position of all joints and move to start position.
     *
     * Homes all joints using home_on_index_after_negative_end_stop.  When
     * finished, move the joint to the starting position (defined in
     * `config_.initial_position_rad`).
     *
     */
    void initialize() override;

    Observation get_latest_observation() override;
    Action apply_action(const Action &desired_action) override;
    std::string get_error() override;
    void shutdown() override;

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

    Action apply_action_uninitialized(const Action &desired_action);

    //! \brief Actual initialization that is called in a real-time thread in
    //!        initialize().
    void _initialize();

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
        Vector home_offset_rad = Vector::Zero());

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
                          const uint32_t timeout_cycles);
};

/**
 * @brief Configuration of the robot that can be changed by the user.
 */
template <size_t N_JOINTS, size_t N_MOTOR_BOARDS>
struct NJointBlmcRobotDriver<N_JOINTS, N_MOTOR_BOARDS>::Config
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
    void print() const;

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
    static Config load_config(const std::string &config_file_name);

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
                                 T *var);
};

template <typename Driver>
typename Driver::Types::BackendPtr create_backend(
    typename Driver::Types::BaseDataPtr robot_data,
    const std::string &config_file_path)
{
    constexpr double MAX_ACTION_DURATION_S = 0.003;
    constexpr double MAX_INTER_ACTION_DURATION_S = 0.005;

    std::cout << "Load robot driver configuration from file '"
              << config_file_path << "'" << std::endl;
    auto config = Driver::Config::load_config(config_file_path);
    config.print();

    // wrap the actual robot driver directly in a MonitoredRobotDriver
    auto monitored_driver =
        std::make_shared<robot_interfaces::MonitoredRobotDriver<Driver>>(
            std::make_shared<Driver>(config),
            MAX_ACTION_DURATION_S,
            MAX_INTER_ACTION_DURATION_S);

    auto backend = std::make_shared<typename Driver::Types::Backend>(
        monitored_driver, robot_data);
    backend->set_max_action_repetitions(std::numeric_limits<uint32_t>::max());

    return backend;
}

}  // namespace blmc_robots

#include "n_joint_blmc_robot_driver.hxx"
