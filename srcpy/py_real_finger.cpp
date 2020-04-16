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

#include <robot_fingers/fake_finger_driver.hpp>
#include <robot_fingers/real_finger_driver.hpp>

using namespace pybind11::literals;
using namespace robot_fingers;
using namespace blmc_robots;

PYBIND11_MODULE(py_real_finger, m)
{
    m.def("create_real_finger_backend",
          &create_backend<RealFingerDriver>,
          "robot_data"_a,
          "config_file"_a,
          "first_action_timeout"_a = std::numeric_limits<double>::infinity());

    m.def("create_fake_finger_backend", &create_fake_finger_backend);
}
