/**
 * @file
 * @brief Combined robot/camera driver that replays a given log.
 * @copyright 2021, Max Planck Gesellschaft.  All rights reserved.
 * @license BSD 3-clause
 */
#include <pybind11/pybind11.h>

#include <robot_fingers/log_replay_driver.hpp>

using namespace robot_fingers;

PYBIND11_MODULE(log_replay_driver, m)
{
    pybind11::class_<TriFingerPlatformLogReplayDriver,
                     std::shared_ptr<TriFingerPlatformLogReplayDriver>>(
        m, "TriFingerPlatformLogReplayDriver")
        .def(pybind11::init<const std::string&, const std::string&>());

    m.def("create_backend",
          &create_log_replay_backend,
          pybind11::arg("driver"),
          pybind11::arg("robot_data"),
          pybind11::arg("first_action_timeout") =
              std::numeric_limits<double>::infinity(),
          pybind11::arg("max_number_of_actions") = 0);
}
