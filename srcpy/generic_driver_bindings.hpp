/**
 * @file
 * @brief Helper functions for Python-binding templated types.
 * @copyright 2019, Max Planck Gesellschaft.  All rights reserved.
 * @license BSD 3-clause
 */
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <robot_interfaces/n_joint_robot_types.hpp>

namespace robot_fingers
{
template <typename Driver>
void bind_driver_config(pybind11::module &m, const std::string &name)
{
    pybind11::class_<typename Driver::Config,
                     std::shared_ptr<typename Driver::Config>>
        config(m, name.c_str());
    config.def(pybind11::init<>())
        .def_static("load_config",
                    &Driver::Config::load_config,
                    pybind11::call_guard<pybind11::gil_scoped_release>(),
                    pybind11::arg("config_file_name"),
                    R"XXX(
             load_config(config_file_name: str) -> Config

             Load driver configuration from a YAML file.

             Args:
                 config_file_name:  Path to the config file.

             Returns:
                 The configuration loaded from the given file.
)XXX")
        .def("is_within_hard_position_limits",
             &Driver::Config::is_within_hard_position_limits,
             pybind11::call_guard<pybind11::gil_scoped_release>(),
             pybind11::arg("position"),
             R"XXX(
             is_within_hard_position_limits(position: list) -> bool

             Check if the given position is within the hard limits.

             Args:
                 position:  Joint positions.

             Returns:
                 True if `hard_position_limits_lower <= position <=
                 hard_position_limits_upper`.
)XXX")
        .def("print",
             &Driver::Config::print,
             pybind11::call_guard<pybind11::gil_scoped_release>(),
             R"XXX(
             print()

             Print the configuration.
)XXX")
        .def_readwrite("can_ports",
                       &Driver::Config::can_ports,
                       "List of CAN port names used by the robot.")
        .def_readwrite("max_current_A",
                       &Driver::Config::max_current_A,
                       "Maximum current that can be sent to the motor [A].")
        .def_readwrite("has_endstop",
                       &Driver::Config::has_endstop,
                       "Whether the joints have physical end stops or not.")
        .def_readwrite("calibration",
                       &Driver::Config::calibration,
                       "Parameters related to calibration.")
        .def_readwrite("move_to_position_tolerance_rad",
                       &Driver::Config::move_to_position_tolerance_rad,
                       "Tolerance for reaching the target with "
                       "NJointBlmcRobotDriver::move_to_position()")
        .def_readwrite(
            "safety_kd",
            &Driver::Config::safety_kd,
            "D-gain to dampen velocity.  Set to zero to disable damping.")
        .def_readwrite("position_control_gains",
                       &Driver::Config::position_control_gains,
                       "Default control gains for the position controller.")
        .def_readwrite("hard_position_limits_lower",
                       &Driver::Config::hard_position_limits_lower,
                       "Hard lower limits for joint positions.")
        .def_readwrite("hard_position_limits_upper",
                       &Driver::Config::hard_position_limits_upper,
                       "Hard upper limits for joint positions.")
        .def_readwrite("soft_position_limits_lower",
                       &Driver::Config::soft_position_limits_lower,
                       "Soft lower limits for joint positions.")
        .def_readwrite("soft_position_limits_upper",
                       &Driver::Config::soft_position_limits_upper,
                       "Soft upper limits for joint positions.")
        .def_readwrite("home_offset_rad",
                       &Driver::Config::home_offset_rad,
                       "Offset between home and zero position.")
        .def_readwrite(
            "initial_position_rad",
            &Driver::Config::initial_position_rad,
            "Initial position to which the robot moves during initialisation.")
        .def_readwrite("shutdown_trajectory",
                       &Driver::Config::shutdown_trajectory,
                       "Trajectory which is executed during shutdown.");

    pybind11::class_<typename Driver::Config::TrajectoryStep>(config,
                                                              "TrajectoryStep")
        .def(pybind11::init<>())
        .def_readwrite("target_position_rad",
                       &Driver::Config::TrajectoryStep::target_position_rad,
                       "Target position to which the joints should move.")
        .def_readwrite(
            "move_steps",
            &Driver::Config::TrajectoryStep::move_steps,
            "Number of time steps for reaching the target position.");

    pybind11::class_<typename Driver::Config::CalibrationParameters>(
        config, "CalibrationParameters")
        .def(pybind11::init<>())
        .def_readwrite(
            "endstop_search_torques_Nm",
            &Driver::Config::CalibrationParameters::endstop_search_torques_Nm,
            "Torque that is used to find the end stop.")
        .def_readwrite(
            "move_steps",
            &Driver::Config::CalibrationParameters::move_steps,
            "Number of time steps for reaching the initial position.");

    pybind11::class_<typename Driver::Config::PositionControlGains>(
        config, "PositionControlGains")
        .def(pybind11::init<>())
        .def_readwrite("kp", &Driver::Config::PositionControlGains::kp)
        .def_readwrite("kd", &Driver::Config::PositionControlGains::kd);
}

template <typename Driver>
void bind_create_backend(pybind11::module &m, const std::string &name)
{
    m.def(name.c_str(),
          pybind11::overload_cast<typename Driver::Types::BaseDataPtr,
                                  const typename Driver::Config &,
                                  const double,
                                  const uint32_t>(&create_backend<Driver>),
          pybind11::arg("robot_data"),
          pybind11::arg("config"),
          pybind11::arg("first_action_timeout") =
              std::numeric_limits<double>::infinity(),
          pybind11::arg("max_number_of_actions") = 0);

    m.def(name.c_str(),
          pybind11::overload_cast<typename Driver::Types::BaseDataPtr,
                                  const std::string &,
                                  const double,
                                  const uint32_t>(&create_backend<Driver>),
          pybind11::arg("robot_data"),
          pybind11::arg("config_file"),
          pybind11::arg("first_action_timeout") =
              std::numeric_limits<double>::infinity(),
          pybind11::arg("max_number_of_actions") = 0);
}

}  // namespace robot_fingers
