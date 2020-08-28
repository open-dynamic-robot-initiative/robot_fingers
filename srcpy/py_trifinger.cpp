/*
 * Copyright [2017] Max Planck Society. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <pybind11/eigen.h>
#include <pybind11/embed.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>

#include <robot_fingers/trifinger_driver.hpp>
#include <robot_fingers/trifinger_platform_frontend.hpp>

using namespace pybind11::literals;
using namespace robot_fingers;
using namespace blmc_robots;

PYBIND11_MODULE(py_trifinger, m)
{
    m.def("create_trifinger_backend",
          &create_backend<TriFingerDriver>,
          "robot_data"_a,
          "config_file"_a,
          "first_action_timeout"_a = std::numeric_limits<double>::infinity(),
          "max_number_of_actions"_a = 0);

    pybind11::class_<TriFingerPlatformFrontend,
                     std::shared_ptr<TriFingerPlatformFrontend>>(
        m, "TriFingerPlatformFrontend")
        .def(pybind11::init<>())
        .def("get_robot_observation",
             &TriFingerPlatformFrontend::get_robot_observation,
             pybind11::call_guard<pybind11::gil_scoped_release>())
        .def("get_camera_observation",
             &TriFingerPlatformFrontend::get_camera_observation,
             pybind11::call_guard<pybind11::gil_scoped_release>())
        .def("get_desired_action",
             &TriFingerPlatformFrontend::get_desired_action,
             pybind11::call_guard<pybind11::gil_scoped_release>())
        .def("get_applied_action",
             &TriFingerPlatformFrontend::get_applied_action,
             pybind11::call_guard<pybind11::gil_scoped_release>())
        .def("get_robot_status",
             &TriFingerPlatformFrontend::get_robot_status,
             pybind11::call_guard<pybind11::gil_scoped_release>())
        .def("get_timestamp_ms",
             &TriFingerPlatformFrontend::get_timestamp_ms,
             pybind11::call_guard<pybind11::gil_scoped_release>())
        .def("append_desired_action",
             &TriFingerPlatformFrontend::append_desired_action,
             pybind11::call_guard<pybind11::gil_scoped_release>())
        .def("wait_until_timeindex",
             &TriFingerPlatformFrontend::wait_until_timeindex,
             pybind11::call_guard<pybind11::gil_scoped_release>())
        .def("get_current_timeindex",
             &TriFingerPlatformFrontend::get_current_timeindex,
             pybind11::call_guard<pybind11::gil_scoped_release>());
}
