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
#include <robot_fingers/trifinger_platform_log.hpp>

using namespace pybind11::literals;
using namespace robot_fingers;

PYBIND11_MODULE(py_trifinger, m)
{
    pybind11::options options;
    // disable automatic function signature generation as this does not look too
    // nice in the Sphinx documentation.
    options.disable_function_signatures();

    // needed for bindings of camera observations
    pybind11::module::import("trifinger_object_tracking.py_tricamera_types");

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
        .def("append_desired_action",
             &TriFingerPlatformFrontend::append_desired_action,
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
             &TriFingerPlatformFrontend::get_robot_observation,
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
        .def("get_camera_observation",
             &TriFingerPlatformFrontend::get_camera_observation,
             pybind11::call_guard<pybind11::gil_scoped_release>(),
             R"XXX(
                get_camera_observation(t: int) -> trifinger_object_tracking.py_tricamera_types.TriCameraObjectObservation

                Get camera images of time step t.

                If t is in the future, this method will block and wait.

                Args:
                    t:  Time index of the robot time series.  This is internally
                        mapped to the corresponding time index of the camera
                        time series.

                Returns:
                    Camera images of time step t.

                Raises:
                    Exception: if t is too old and not in the time series buffer
                        anymore.
)XXX")
        .def("get_desired_action",
             &TriFingerPlatformFrontend::get_desired_action,
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
             &TriFingerPlatformFrontend::get_applied_action,
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
             &TriFingerPlatformFrontend::get_robot_status,
             pybind11::call_guard<pybind11::gil_scoped_release>(),
             R"XXX(
                get_robot_status(t: int) -> robot_interfaces.Status

                Get robot status of time step t.

                If t is in the future, this method will block and wait.
)XXX")
        .def("get_timestamp_ms",
             &TriFingerPlatformFrontend::get_timestamp_ms,
             pybind11::call_guard<pybind11::gil_scoped_release>(),
             R"XXX(
                get_timestamp_ms(t: int) -> float

                Get timestamp in milliseconds of time step t.

                If t is in the future, this method will block and wait.
)XXX")
        .def("wait_until_timeindex",
             &TriFingerPlatformFrontend::wait_until_timeindex,
             pybind11::call_guard<pybind11::gil_scoped_release>(),
             R"XXX(
                wait_until_timeindex(t: int)

                Wait until time step t is reached.
)XXX")
        .def("get_current_timeindex",
             &TriFingerPlatformFrontend::get_current_timeindex,
             pybind11::call_guard<pybind11::gil_scoped_release>(),
             R"XXX(
                get_current_timeindex() -> int

                Get the current time index.
)XXX");

    pybind11::class_<TriFingerPlatformLog,
                     std::shared_ptr<TriFingerPlatformLog>>(
        m,
        "TriFingerPlatformLog",
        R"XXX(
        TriFingerPlatformLog(robot_log_file: str, camera_log_file: str)

        Load robot and camera log and match observations like during runtime.

        The robot and camera observations are provided asynchronously.  To
        access both through a common time index,
        :class:`TriFingerPlatformFrontend` maps "robot time indices" to the
        corresponding camera observations based on the time stamps.  This
        mapping is not explicitly saved in the log files.  Therefore,
        TriFingerPlatformLog class provides an interface to load robot and
        camera logs together and performs the mapping from robot to camera time
        index in the same way as it is happening in
        :class:`TriFingerPlatformFrontend`.

        Args:
            robot_log_file (str): Path to the robot log file.
            camera_log_file (str): Path to the camera log file.
)XXX")
        .def(pybind11::init<const std::string&, const std::string&>(),
             "robot_log_file"_a,
             "camera_log_file"_a)
        .def("get_robot_observation",
             &TriFingerPlatformLog::get_robot_observation,
             pybind11::call_guard<pybind11::gil_scoped_release>(),
             "t"_a,
             R"XXX(
                get_robot_observation(t: int) -> Observation

                Get robot observation of time step t.
)XXX")
        .def("get_camera_observation",
             &TriFingerPlatformLog::get_camera_observation,
             pybind11::call_guard<pybind11::gil_scoped_release>(),
             "t"_a,
             R"XXX(
                get_camera_observation(t: int) -> trifinger_object_tracking.py_tricamera_types.TriCameraObjectObservation

                Get camera observation of robot time step t.
)XXX")
        .def("get_desired_action",
             &TriFingerPlatformLog::get_desired_action,
             pybind11::call_guard<pybind11::gil_scoped_release>(),
             "t"_a,
             R"XXX(
                get_desired_action(t: int) -> Action

                Get desired action of time step t.
)XXX")
        .def("get_applied_action",
             &TriFingerPlatformLog::get_applied_action,
             pybind11::call_guard<pybind11::gil_scoped_release>(),
             "t"_a,
             R"XXX(
                get_applied_action(t: int) -> Action

                Get actually applied action of time step t.
)XXX")
        .def("get_robot_status",
             &TriFingerPlatformLog::get_robot_status,
             pybind11::call_guard<pybind11::gil_scoped_release>(),
             "t"_a,
             R"XXX(
                get_robot_status(t: int) -> Status

                Get robot status of time step t.
)XXX")
        .def("get_timestamp_ms",
             &TriFingerPlatformLog::get_timestamp_ms,
             pybind11::call_guard<pybind11::gil_scoped_release>(),
             "t"_a,
             R"XXX(
                get_timestamp_ms(t: int) -> float

                Get timestamp (in milliseconds) of time step t.
)XXX")
        .def("get_first_timeindex",
             &TriFingerPlatformLog::get_first_timeindex,
             pybind11::call_guard<pybind11::gil_scoped_release>(),
             R"XXX(
                get_first_timeindex() -> int

                Get the first time index in the log file.
)XXX")
        .def("get_last_timeindex",
             &TriFingerPlatformLog::get_last_timeindex,
             pybind11::call_guard<pybind11::gil_scoped_release>(),
             R"XXX(
                get_last_timeindex() -> int

                Get the last time index in the log file.
)XXX");
}
