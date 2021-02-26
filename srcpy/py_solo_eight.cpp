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
// #include <robot_fingers/trifinger_driver.hpp>

#include <pybind11/eigen.h>
#include <pybind11/embed.h>
#include <pybind11/stl_bind.h>
#include <pybind11/pybind11.h>

#include <robot_fingers/solo_eight_driver.hpp>
#include <robot_fingers/solo_eight_platform_frontend.hpp>

#include "generic_driver_bindings.hpp"

using namespace pybind11::literals;
using namespace robot_fingers;

PYBIND11_MODULE(py_solo_eight, m)
{
    pybind11::options options;
    // disable automatic function signature generation as this does not look too
    // nice in the Sphinx documentation.
    options.disable_function_signatures();

    bind_create_backend<SoloEightDriver>(m, "create_solo_eight_backend");
    bind_driver_config<SoloEightDriver>(m, "SoloEightConfig");

    pybind11::class_<SoloEightPlatformFrontend,
                     std::shared_ptr<SoloEightPlatformFrontend>>(
        m, "SoloEightPlatformFrontend")
        .def(pybind11::init<>())
        .def("append_desired_action",
             &SoloEightPlatformFrontend::append_desired_action,
             pybind11::call_guard<pybind11::gil_scoped_release>(),
             "desired_action"_a,
             R"XXX(
             append_desired_action(action: robot_interfaces.trifinger.Action) -> int

             Append a desired action to the action time series.

             Append an action to the "desired actions" time series.
             Note that this does *not* block until the action is actually
             executed. The time series acts like a queue from which the
             actions are taken one by one to send them to the actual robot.
             It is possible to call this method multiple times in a row to
             already provide actions for the next time steps.

             The time step at which the given action will be applied is
             returned by this method.

             Args:
                 desired_action:  The action that shall be applied on the
                     robot.  Note that the actually applied action might be
                     different (see :meth:`get_applied_action`).

             Returns:
                 Time step at which the action will be applied.
)XXX")
        .def("get_robot_observation",
             &SoloEightPlatformFrontend::get_robot_observation,
             pybind11::call_guard<pybind11::gil_scoped_release>(),
             R"XXX(
                get_robot_observation(t: int) -> robot_interfaces.trifinger.Observation

                Get robot observation of time step t.

                If t is in the future, this method will block and wait.

                Args:
                    t: Index of the time step.

                Returns:
                    Robot observation of time step t.

                Raises:
                    Exception: if t is too old and not in the time series buffer
                        anymore.
)XXX")
        .def("get_desired_action",
             &SoloEightPlatformFrontend::get_desired_action,
             pybind11::call_guard<pybind11::gil_scoped_release>(),
             R"XXX(
                get_desired_action(t: int) -> robot_interfaces.trifinger.Action

                Get desired action of time step t.

                This corresponds to the action that was appended using
                `append_desired_action()`.  Note that the actually applied
                action may differ, see `get_applied_action()`.

                If t is in the future, this method will block and wait.
)XXX")
        .def("get_applied_action",
             &SoloEightPlatformFrontend::get_applied_action,
             pybind11::call_guard<pybind11::gil_scoped_release>(),
             R"XXX(
                get_applied_action(t: int) -> robot_interfaces.trifinger.Action

                Get actually applied action of time step t.

                The applied action is the one that was actually applied to the
                robot based on the desired action of that time step.  It may
                differ from the desired one e.g. due to safety checks which
                limit the maximum torque and enforce the position limits.

                If t is in the future, this method will block and wait.
)XXX")
        .def("get_robot_status",
             &SoloEightPlatformFrontend::get_robot_status,
             pybind11::call_guard<pybind11::gil_scoped_release>(),
             R"XXX(
                get_robot_status(t: int) -> robot_interfaces.Status

                Get robot status of time step t.

                If t is in the future, this method will block and wait.
)XXX")
        .def("get_timestamp_ms",
             &SoloEightPlatformFrontend::get_timestamp_ms,
             pybind11::call_guard<pybind11::gil_scoped_release>(),
             R"XXX(
                get_timestamp_ms(t: int) -> float

                Get timestamp in milliseconds of time step t.

                If t is in the future, this method will block and wait.
)XXX")
        .def("wait_until_timeindex",
             &SoloEightPlatformFrontend::wait_until_timeindex,
             pybind11::call_guard<pybind11::gil_scoped_release>(),
             R"XXX(
                wait_until_timeindex(t: int)

                Wait until time step t is reached.
)XXX")
        .def("get_current_timeindex",
             &SoloEightPlatformFrontend::get_current_timeindex,
             pybind11::call_guard<pybind11::gil_scoped_release>(),
             R"XXX(
                get_current_timeindex() -> int

                Get the current time index.
)XXX");
}
