/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <mrpt/kinematics/CVehicleSimul_DiffDriven.h>
#include <mrpt/kinematics/CVehicleSimul_Holo.h>
#include <mrpt/kinematics/CVehicleVelCmd_DiffDriven.h>
#include <mrpt/kinematics/CVehicleVelCmd_Holo.h>
#include <mrpt/math/TPose2D.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_MODULE(_bindings, m)
{
  m.doc() = "Python bindings for mrpt::kinematics — vehicle kinematic models and simulators";

  // -------------------------------------------------------------------------
  // CVehicleVelCmd_DiffDriven — velocity command for differential-drive robots
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::kinematics::CVehicleVelCmd_DiffDriven,
      std::shared_ptr<mrpt::kinematics::CVehicleVelCmd_DiffDriven>>(m, "CVehicleVelCmd_DiffDriven")
      .def(py::init<>())
      .def_readwrite(
          "lin_vel", &mrpt::kinematics::CVehicleVelCmd_DiffDriven::lin_vel, "Linear velocity (m/s)")
      .def_readwrite(
          "ang_vel", &mrpt::kinematics::CVehicleVelCmd_DiffDriven::ang_vel,
          "Angular velocity (rad/s)")
      .def("isStopCmd", &mrpt::kinematics::CVehicleVelCmd_DiffDriven::isStopCmd)
      .def("setToStop", &mrpt::kinematics::CVehicleVelCmd_DiffDriven::setToStop)
      .def(
          "__repr__",
          [](const mrpt::kinematics::CVehicleVelCmd_DiffDriven& cmd)
          {
            return "CVehicleVelCmd_DiffDriven(lin=" + std::to_string(cmd.lin_vel) +
                   ", ang=" + std::to_string(cmd.ang_vel) + ")";
          });

  // -------------------------------------------------------------------------
  // CVehicleVelCmd_Holo — velocity command for holonomic robots
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::kinematics::CVehicleVelCmd_Holo,
      std::shared_ptr<mrpt::kinematics::CVehicleVelCmd_Holo>>(m, "CVehicleVelCmd_Holo")
      .def(py::init<>())
      .def(
          py::init<double, double, double, double>(), "vel"_a, "dir_local"_a, "ramp_time"_a,
          "rot_speed"_a)
      .def_readwrite("vel", &mrpt::kinematics::CVehicleVelCmd_Holo::vel, "Linear speed (m/s)")
      .def_readwrite(
          "dir_local", &mrpt::kinematics::CVehicleVelCmd_Holo::dir_local,
          "Direction relative to robot heading (radians)")
      .def_readwrite(
          "ramp_time", &mrpt::kinematics::CVehicleVelCmd_Holo::ramp_time, "Blending time (seconds)")
      .def_readwrite(
          "rot_speed", &mrpt::kinematics::CVehicleVelCmd_Holo::rot_speed,
          "Rotational speed for heading correction (rad/s)")
      .def("isStopCmd", &mrpt::kinematics::CVehicleVelCmd_Holo::isStopCmd)
      .def("setToStop", &mrpt::kinematics::CVehicleVelCmd_Holo::setToStop)
      .def(
          "__repr__",
          [](const mrpt::kinematics::CVehicleVelCmd_Holo& cmd)
          {
            return "CVehicleVelCmd_Holo(vel=" + std::to_string(cmd.vel) +
                   ", dir=" + std::to_string(cmd.dir_local) + ")";
          });

  // -------------------------------------------------------------------------
  // CVehicleSimulVirtualBase — abstract base for vehicle simulators
  // -------------------------------------------------------------------------
  py::class_<mrpt::kinematics::CVehicleSimulVirtualBase>(m, "CVehicleSimulVirtualBase")
      .def(
          "simulateOneTimeStep", &mrpt::kinematics::CVehicleSimulVirtualBase::simulateOneTimeStep,
          "dt"_a, "Advance simulation by dt seconds")
      .def(
          "getCurrentGTPose",
          [](const mrpt::kinematics::CVehicleSimulVirtualBase& s) -> const mrpt::math::TPose2D&
          { return s.getCurrentGTPose(); },
          py::return_value_policy::reference_internal, "Get current ground-truth pose (x, y, phi)")
      .def(
          "getCurrentOdometricPose",
          [](const mrpt::kinematics::CVehicleSimulVirtualBase& s) -> const mrpt::math::TPose2D&
          { return s.getCurrentOdometricPose(); },
          py::return_value_policy::reference_internal,
          "Get current odometric (noisy) pose (x, y, phi)")
      .def(
          "setCurrentGTPose",
          [](mrpt::kinematics::CVehicleSimulVirtualBase& s, const mrpt::math::TPose2D& p)
          { s.setCurrentGTPose(p); },
          "pose"_a)
      .def(
          "setCurrentOdometricPose",
          [](mrpt::kinematics::CVehicleSimulVirtualBase& s, const mrpt::math::TPose2D& p)
          { s.setCurrentOdometricPose(p); },
          "pose"_a);

  // -------------------------------------------------------------------------
  // CVehicleSimul_DiffDriven — differential-drive robot simulator
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::kinematics::CVehicleSimul_DiffDriven, mrpt::kinematics::CVehicleSimulVirtualBase>(
      m, "CVehicleSimul_DiffDriven")
      .def(py::init<>())
      .def(
          "movementCommand", &mrpt::kinematics::CVehicleSimul_DiffDriven::movementCommand,
          "lin_vel"_a, "ang_vel"_a, "Set velocity command: linear (m/s) and angular (rad/s)")
      .def(
          "__repr__",
          [](const mrpt::kinematics::CVehicleSimul_DiffDriven& s)
          {
            const auto& p = s.getCurrentGTPose();
            return "CVehicleSimul_DiffDriven(x=" + std::to_string(p.x) +
                   ", y=" + std::to_string(p.y) + ", phi=" + std::to_string(p.phi) + ")";
          });

  // -------------------------------------------------------------------------
  // CVehicleSimul_Holo — holonomic robot simulator
  // -------------------------------------------------------------------------
  py::class_<mrpt::kinematics::CVehicleSimul_Holo, mrpt::kinematics::CVehicleSimulVirtualBase>(
      m, "CVehicleSimul_Holo")
      .def(py::init<>())
      .def(
          "sendVelRampCmd", &mrpt::kinematics::CVehicleSimul_Holo::sendVelRampCmd, "vel"_a, "dir"_a,
          "ramp_time"_a, "rot_speed"_a, "Send a velocity ramp command to the holonomic robot")
      .def(
          "__repr__",
          [](const mrpt::kinematics::CVehicleSimul_Holo& s)
          {
            const auto& p = s.getCurrentGTPose();
            return "CVehicleSimul_Holo(x=" + std::to_string(p.x) + ", y=" + std::to_string(p.y) +
                   ", phi=" + std::to_string(p.phi) + ")";
          });
}
