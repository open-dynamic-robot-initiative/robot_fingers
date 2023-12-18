#include <pybind11/embed.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>

#include <robot_fingers/pybullet_driver.hpp>
#include <robot_interfaces/finger_types.hpp>

using namespace pybind11::literals;

PYBIND11_MODULE(pybullet_drivers, m)
{
    m.def("create_single_finger_backend",
          &trifinger_simulation::create_finger_backend<
              robot_interfaces::MonoFingerTypes,
              trifinger_simulation::PyBulletSingleFingerDriver>,
          "robot_data"_a,
          "real_time_mode"_a,
          "visualize"_a,
          "first_action_timeout"_a = std::numeric_limits<double>::infinity(),
          "max_number_of_actions"_a = 0,
          R"XXX(
            Create backend for the Single Finger robot using PyBullet simulation.

            Args:
                robot_data (robot_interfaces.finger.Data):  Robot data instance
                    for the Finger robot.
                real_time_mode (bool):  If True, step the simulation in real
                    time, otherwise as fast as possible.
                visualize (bool):  If True, the PyBullet GUI is started for
                    visualization.
                first_action_timeout (float): Timeout for the first action to
                    arrive.  If exceeded, the backend shuts down.  Set to
                    infinity to disable the timeout.
                max_number_of_actions (int): Maximum number of actions that are
                    executed by the backend.  If set to a value greater than
                    zero, the backend will automatically shut down after the
                    specified number of actions is executed.

            Returns:
                Finger backend using simulation instead of the real robot.
)XXX");

    m.def("create_trifinger_backend",
          &trifinger_simulation::create_finger_backend<
              robot_interfaces::TriFingerTypes,
              trifinger_simulation::PyBulletTriFingerDriver>,
          "robot_data"_a,
          "real_time_mode"_a,
          "visualize"_a,
          "first_action_timeout"_a = std::numeric_limits<double>::infinity(),
          "max_number_of_actions"_a = 0,
          R"XXX(
            Create a backend for the TriFinger robot using PyBullet simulation.

            Args:
                robot_data (robot_interfaces.trifinger.Data):  Robot data instance
                    for the TriFinger robot.
                real_time_mode (bool):  If True, step the simulation in real
                    time, otherwise as fast as possible.
                visualize (bool):  If True, the PyBullet GUI is started for
                    visualization.
                first_action_timeout (float): Timeout for the first action to
                    arrive.  If exceeded, the backend shuts down.  Set to
                    infinity to disable the timeout.
                max_number_of_actions (int): Maximum number of actions that are
                    executed by the backend.  If set to a value greater than
                    zero, the backend will automatically shut down after the
                    specified number of actions is executed.

            Returns:
                TriFinger backend using simulation instead of the real robot.
)XXX");
}
