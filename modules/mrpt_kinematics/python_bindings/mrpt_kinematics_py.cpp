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
#include <mrpt/math/TTwist2D.h>
#include <mrpt/serialization/CSerializable.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_MODULE(_bindings, m)
{
  m.doc() = "Python bindings for mrpt::kinematics: vehicle kinematic models and simulators";

  // -------------------------------------------------------------------------
  // CVehicleVelCmd: base class of velocity commands
  // -------------------------------------------------------------------------
  using mrpt::kinematics::CVehicleVelCmd;
  py::class_<CVehicleVelCmd, mrpt::serialization::CSerializable, std::shared_ptr<CVehicleVelCmd>>(
      m, "CVehicleVelCmd",
      "Base class of the velocity commands of the kinematic models (and of real robots).")
      .def(
          "getVelCmdLength", &CVehicleVelCmd::getVelCmdLength,
          "Returns the number of components of the velocity command.")
      .def(
          "getVelCmdDescription", &CVehicleVelCmd::getVelCmdDescription, "index"_a,
          "Returns a description of the index-th component of the velocity command.")
      .def(
          "getVelCmdElement", &CVehicleVelCmd::getVelCmdElement, "index"_a,
          "Returns the index-th component of the velocity command.")
      .def(
          "setVelCmdElement", &CVehicleVelCmd::setVelCmdElement, "index"_a, "value"_a,
          "Sets the index-th component of the velocity command.")
      .def(
          "asString", &CVehicleVelCmd::asString,
          "Returns a human-readable description of the command.")
      .def("__str__", &CVehicleVelCmd::asString);

  // -------------------------------------------------------------------------
  // CVehicleVelCmd_DiffDriven: velocity command for differential-drive robots
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::kinematics::CVehicleVelCmd_DiffDriven, CVehicleVelCmd,
      std::shared_ptr<mrpt::kinematics::CVehicleVelCmd_DiffDriven>>(
      m, "CVehicleVelCmd_DiffDriven",
      "Kinematic model for Ackermann-like or differential-driven vehicles.")
      .def(py::init<>(), "Default constructor.")
      .def_readwrite(
          "lin_vel", &mrpt::kinematics::CVehicleVelCmd_DiffDriven::lin_vel, "Linear velocity (m/s)")
      .def_readwrite(
          "ang_vel", &mrpt::kinematics::CVehicleVelCmd_DiffDriven::ang_vel,
          "Angular velocity (rad/s)")
      .def(
          "isStopCmd", &mrpt::kinematics::CVehicleVelCmd_DiffDriven::isStopCmd,
          "Returns true if the command means \"do not move\" / \"stop\".")
      .def(
          "setToStop", &mrpt::kinematics::CVehicleVelCmd_DiffDriven::setToStop,
          "Set to a command that means \"do not move\" / \"stop\".")
      .def(
          "__repr__",
          [](const mrpt::kinematics::CVehicleVelCmd_DiffDriven& cmd)
          {
            return "CVehicleVelCmd_DiffDriven(lin=" + std::to_string(cmd.lin_vel) +
                   ", ang=" + std::to_string(cmd.ang_vel) + ")";
          });

  // -------------------------------------------------------------------------
  // CVehicleVelCmd_Holo: velocity command for holonomic robots
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::kinematics::CVehicleVelCmd_Holo, CVehicleVelCmd,
      std::shared_ptr<mrpt::kinematics::CVehicleVelCmd_Holo>>(
      m, "CVehicleVelCmd_Holo",
      "Velocity command for holonomic robots: speed, direction, rotational speed and ramp time.")
      .def(py::init<>(), "Default constructor.")
      .def(
          py::init<double, double, double, double>(), "vel"_a, "dir_local"_a, "ramp_time"_a,
          "rot_speed"_a,
          "Builds a command from speed, direction (rad), ramp time (s) and rotational speed "
          "(rad/s).")
      .def_readwrite("vel", &mrpt::kinematics::CVehicleVelCmd_Holo::vel, "Linear speed (m/s)")
      .def_readwrite(
          "dir_local", &mrpt::kinematics::CVehicleVelCmd_Holo::dir_local,
          "Direction relative to robot heading (radians)")
      .def_readwrite(
          "ramp_time", &mrpt::kinematics::CVehicleVelCmd_Holo::ramp_time, "Blending time (seconds)")
      .def_readwrite(
          "rot_speed", &mrpt::kinematics::CVehicleVelCmd_Holo::rot_speed,
          "Rotational speed for heading correction (rad/s)")
      .def(
          "isStopCmd", &mrpt::kinematics::CVehicleVelCmd_Holo::isStopCmd,
          "Returns true if the command means \"do not move\" / \"stop\".")
      .def(
          "setToStop", &mrpt::kinematics::CVehicleVelCmd_Holo::setToStop,
          "Set to a command that means \"do not move\" / \"stop\".")
      .def(
          "__repr__",
          [](const mrpt::kinematics::CVehicleVelCmd_Holo& cmd)
          {
            return "CVehicleVelCmd_Holo(vel=" + std::to_string(cmd.vel) +
                   ", dir=" + std::to_string(cmd.dir_local) + ")";
          });

  // -------------------------------------------------------------------------
  // CVehicleSimulVirtualBase: abstract base for vehicle simulators
  // -------------------------------------------------------------------------
  py::class_<mrpt::kinematics::CVehicleSimulVirtualBase>(
      m, "CVehicleSimulVirtualBase",
      "Base class of the 2D robot kinematic simulators, including odometry errors.")
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
          "pose"_a, "Brute-force move robot to target coordinates (\"teleport\")")
      .def(
          "setCurrentOdometricPose",
          [](mrpt::kinematics::CVehicleSimulVirtualBase& s, const mrpt::math::TPose2D& p)
          { s.setCurrentOdometricPose(p); },
          "pose"_a, "Brute-force overwrite robot odometry.")
      .def(
          "getCurrentGTVel",
          [](const mrpt::kinematics::CVehicleSimulVirtualBase& s) { return s.getCurrentGTVel(); },
          "Returns the ground truth velocity (vx, vy, omega) in world coordinates. With odometry "
          "errors enabled, this is the noise-free commanded velocity.")
      .def(
          "getCurrentGTVelLocal", &mrpt::kinematics::CVehicleSimulVirtualBase::getCurrentGTVelLocal,
          "Returns the ground truth velocity (vx, vy, omega) in the robot local frame.")
      .def(
          "getCurrentOdometricVel",
          [](const mrpt::kinematics::CVehicleSimulVirtualBase& s)
          { return s.getCurrentOdometricVel(); },
          "Returns the odometric velocity (vx, vy, omega) in the odometry frame.")
      .def(
          "getCurrentOdometricVelLocal",
          &mrpt::kinematics::CVehicleSimulVirtualBase::getCurrentOdometricVelLocal,
          "Returns the odometric velocity (vx, vy, omega) in the robot local frame.")
      .def(
          "getTime", &mrpt::kinematics::CVehicleSimulVirtualBase::getTime,
          "Returns the current simulation time (seconds).")
      .def(
          "sendVelCmd", &mrpt::kinematics::CVehicleSimulVirtualBase::sendVelCmd, "cmd_vel"_a,
          "Sends a velocity command, whose type must match the simulator: "
          "CVehicleVelCmd_DiffDriven "
          "or CVehicleVelCmd_Holo.")
      .def(
          "getVelCmdType", &mrpt::kinematics::CVehicleSimulVirtualBase::getVelCmdType,
          "Returns an empty velocity command of the type this simulator accepts.")
      .def(
          "setOdometryErrors", &mrpt::kinematics::CVehicleSimulVirtualBase::setOdometryErrors,
          "enabled"_a, "Ax_err_bias"_a = 1e-3, "Ax_err_std"_a = 10e-3, "Ay_err_bias"_a = 1e-3,
          "Ay_err_std"_a = 10e-3, "Aphi_err_bias"_a = mrpt::DEG2RAD(1e-3),
          "Aphi_err_std"_a = mrpt::DEG2RAD(10e-3),
          "Enables or disables odometry errors: relative factors applied to the displacement in "
          "the vehicle frame (x forward, y left, phi heading). Bias values are relative errors; "
          "std values are 1-sigma relative errors accumulated over 1 second of motion.")
      .def(
          "resetStatus", &mrpt::kinematics::CVehicleSimulVirtualBase::resetStatus,
          "Resets the poses, velocities and pending commands, but not the simulation time.")
      .def(
          "resetTime", &mrpt::kinematics::CVehicleSimulVirtualBase::resetTime,
          "Resets the simulation time to zero.");

  // -------------------------------------------------------------------------
  // CVehicleSimul_DiffDriven: differential-drive robot simulator
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::kinematics::CVehicleSimul_DiffDriven, mrpt::kinematics::CVehicleSimulVirtualBase>(
      m, "CVehicleSimul_DiffDriven",
      "Simulates the kinematics of a differential-driven planar mobile robot/vehicle, including "
      "odometry errors and dynamics limitations.")
      .def(py::init<>(), "Default constructor.")
      .def(
          "movementCommand", &mrpt::kinematics::CVehicleSimul_DiffDriven::movementCommand,
          "lin_vel"_a, "ang_vel"_a, "Set velocity command: linear (m/s) and angular (rad/s)")
      .def(
          "setDelayModelParams", &mrpt::kinematics::CVehicleSimul_DiffDriven::setDelayModelParams,
          "TAU_delay_sec"_a = 1.8, "CMD_delay_sec"_a = 0.0,
          "Sets the dynamics model: velocities follow commands with a first order response of "
          "time constant TAU_delay_sec, after a pure delay of CMD_delay_sec (0, 0: immediate).")
      .def(
          "getV", &mrpt::kinematics::CVehicleSimul_DiffDriven::getV,
          "Returns the current linear velocity (m/s).")
      .def(
          "getW", &mrpt::kinematics::CVehicleSimul_DiffDriven::getW,
          "Returns the current angular velocity (rad/s).")
      .def(
          "__repr__",
          [](const mrpt::kinematics::CVehicleSimul_DiffDriven& s)
          {
            const auto& p = s.getCurrentGTPose();
            return "CVehicleSimul_DiffDriven(x=" + std::to_string(p.x) +
                   ", y=" + std::to_string(p.y) + ", phi=" + std::to_string(p.phi) + ")";
          });

  // -------------------------------------------------------------------------
  // CVehicleSimul_Holo: holonomic robot simulator
  // -------------------------------------------------------------------------
  py::class_<mrpt::kinematics::CVehicleSimul_Holo, mrpt::kinematics::CVehicleSimulVirtualBase>(
      m, "CVehicleSimul_Holo",
      "Kinematic simulator of a holonomic 2D robot capable of moving in any direction, with "
      "\"blended\" velocity profiles.")
      .def(py::init<>(), "Default constructor.")
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
