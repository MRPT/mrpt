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

#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/hwdrivers/CGPSInterface.h>
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/hwdrivers/CHokuyoURG.h>
#include <mrpt/hwdrivers/CJoystick.h>
#include <mrpt/hwdrivers/CRoboPeakLidar.h>
#include <mrpt/hwdrivers/CTaoboticsIMU.h>
#include <mrpt/hwdrivers/CVelodyneScanner.h>
#include <mrpt/hwdrivers/registerAllClasses.h>
#include <mrpt/serialization/CSerializable.h>
#include <pybind11/chrono.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_MODULE(_bindings, m)
{
  m.doc() = "Python bindings for mrpt::hwdrivers: sensor drivers";

  mrpt::hwdrivers::registerAllClasses_mrpt_hwdrivers();

  // -------------------------------------------------------------------------
  // CGenericSensor: common interface of all sensor drivers
  // -------------------------------------------------------------------------
  using mrpt::hwdrivers::CGenericSensor;
  py::class_<CGenericSensor, std::shared_ptr<CGenericSensor>> sensor(m, "CGenericSensor");

  py::enum_<CGenericSensor::TSensorState>(sensor, "TSensorState")
      .value("ssInitializing", CGenericSensor::ssInitializing)
      .value("ssWorking", CGenericSensor::ssWorking)
      .value("ssError", CGenericSensor::ssError)
      .value("ssUninitialized", CGenericSensor::ssUninitialized)
      .export_values();

  sensor
      .def_static(
          // Returns the raw pointer so pybind11 builds the holder from the
          // most-derived address: most drivers have CGenericSensor as a
          // non-first base, and a shared_ptr<CGenericSensor> would be reused
          // with the wrong pointer offset.
          "createSensor",
          [](const std::string& className) { return CGenericSensor::createSensor(className); },
          py::return_value_policy::take_ownership, "className"_a,
          "Creates a sensor driver by its class name (e.g. 'CGPSInterface'), or returns None if "
          "the class is unknown. Configure it with loadConfig().")
      .def(
          "loadConfig", &CGenericSensor::loadConfig, "configSource"_a, "section"_a,
          "Loads the sensor parameters from a config file section")
      .def(
          "initialize", &CGenericSensor::initialize, py::call_guard<py::gil_scoped_release>(),
          "Opens the device and prepares it for capturing (call after configuring)")
      .def(
          "doProcess", &CGenericSensor::doProcess, py::call_guard<py::gil_scoped_release>(),
          "Reads from the device. Call it periodically, e.g. at getProcessRate() Hz.")
      .def(
          "getObservations",
          [](CGenericSensor& s)
          {
            std::vector<
                std::pair<mrpt::system::TTimeStamp, mrpt::serialization::CSerializable::Ptr>>
                out;
            for (const auto& [t, obj] : s.getObservations())
            {
              out.emplace_back(t, obj);
            }
            return out;
          },
          "Returns (and removes) the observations gathered so far, as a list of "
          "(timestamp, observation) tuples")
      .def("getState", &CGenericSensor::getState)
      .def("getProcessRate", &CGenericSensor::getProcessRate, "Suggested doProcess() rate (Hz)")
      .def("getSensorLabel", &CGenericSensor::getSensorLabel)
      .def("setSensorLabel", &CGenericSensor::setSensorLabel, "sensorLabel"_a)
      .def("enableVerbose", &CGenericSensor::enableVerbose, "enabled"_a = true)
      .def(
          "setPathForExternalImages", &CGenericSensor::setPathForExternalImages, "directory"_a,
          "For camera sensors: directory where to save images as external files")
      .def(
          "getClassName",
          [](const CGenericSensor& s) { return std::string(s.GetRuntimeClass()->className); })
      .def(
          "__repr__",
          [](const CGenericSensor& s) {
            return std::string(s.GetRuntimeClass()->className) + "(label='" + s.getSensorLabel() +
                   "')";
          });

  // -------------------------------------------------------------------------
  // Specific drivers, for configuration without a config file.
  // py::multiple_inheritance() is required where CGenericSensor is not the
  // first C++ base (COutputLogger comes first): with a single registered base,
  // pybind11 would otherwise cast without adjusting the pointer offset.
  // -------------------------------------------------------------------------
  using mrpt::hwdrivers::CTaoboticsIMU;
  py::class_<CTaoboticsIMU, CGenericSensor, std::shared_ptr<CTaoboticsIMU>>(m, "CTaoboticsIMU")
      .def(py::init<>())
      .def("setSerialPort", &CTaoboticsIMU::setSerialPort, "serialPort"_a)
      .def("setSerialBaudRate", &CTaoboticsIMU::setSerialBaudRate, "rate"_a);

  using mrpt::hwdrivers::CGPSInterface;
  py::class_<CGPSInterface, CGenericSensor, std::shared_ptr<CGPSInterface>>(
      m, "CGPSInterface", py::multiple_inheritance())
      .def(py::init<>())
      .def("setSerialPortName", &CGPSInterface::setSerialPortName, "COM_port"_a)
      .def("getSerialPortName", &CGPSInterface::getSerialPortName)
      .def("setSetupCommands", &CGPSInterface::setSetupCommands, "cmds"_a)
      .def("setShutdownCommands", &CGPSInterface::setShutdownCommands, "cmds"_a)
      .def("setSetupCommandsDelay", &CGPSInterface::setSetupCommandsDelay, "delay_secs"_a);

  using mrpt::hwdrivers::CHokuyoURG;
  py::class_<CHokuyoURG, CGenericSensor, std::shared_ptr<CHokuyoURG>>(
      m, "CHokuyoURG", py::multiple_inheritance())
      .def(py::init<>())
      .def("setSerialPort", &CHokuyoURG::setSerialPort, "port_name"_a)
      .def("setIPandPort", &CHokuyoURG::setIPandPort, "ip"_a, "port"_a)
      .def("setReducedFOV", &CHokuyoURG::setReducedFOV, "fov"_a)
      .def("setScanInterval", &CHokuyoURG::setScanInterval, "skipScanCount"_a);

  using mrpt::hwdrivers::CRoboPeakLidar;
  py::class_<CRoboPeakLidar, CGenericSensor, std::shared_ptr<CRoboPeakLidar>>(
      m, "CRoboPeakLidar", py::multiple_inheritance())
      .def(py::init<>())
      .def("setSerialPort", &CRoboPeakLidar::setSerialPort, "port_name"_a);

  using mrpt::hwdrivers::CVelodyneScanner;
  py::class_<CVelodyneScanner, CGenericSensor, std::shared_ptr<CVelodyneScanner>> velodyne(
      m, "CVelodyneScanner");
  py::enum_<CVelodyneScanner::model_t>(velodyne, "model_t")
      .value("VLP16", CVelodyneScanner::VLP16)
      .value("HDL32", CVelodyneScanner::HDL32)
      .value("HDL64", CVelodyneScanner::HDL64)
      .export_values();
  velodyne.def(py::init<>())
      .def("setModelName", &CVelodyneScanner::setModelName, "model"_a)
      .def("setDeviceIP", &CVelodyneScanner::setDeviceIP, "ip"_a)
      .def(
          "setPCAPInputFile", &CVelodyneScanner::setPCAPInputFile, "pcap_file"_a,
          "Replays packets from a PCAP file instead of reading from the network");

  // -------------------------------------------------------------------------
  // CJoystick: joysticks and gamepads
  // -------------------------------------------------------------------------
  using mrpt::hwdrivers::CJoystick;
  py::class_<CJoystick> joystick(m, "CJoystick");
  py::class_<CJoystick::State>(joystick, "State")
      .def(py::init<>())
      .def_readwrite("buttons", &CJoystick::State::buttons)
      .def_readwrite("axes", &CJoystick::State::axes, "Normalized axis positions")
      .def_readwrite("axes_raw", &CJoystick::State::axes_raw, "Raw axis positions");
  joystick.def(py::init<>())
      .def_static("getJoysticksCount", &CJoystick::getJoysticksCount)
      .def(
          "getJoystickPosition",
          [](CJoystick& j, int nJoy) -> std::optional<CJoystick::State>
          {
            CJoystick::State st;
            if (!j.getJoystickPosition(nJoy, st))
            {
              return std::nullopt;
            }
            return st;
          },
          "nJoy"_a = 0, "Reads the state of joystick nJoy, or None on error")
      .def(
          "setLimits", &CJoystick::setLimits, "minPerAxis"_a, "maxPerAxis"_a,
          "Sets the raw range of each axis, used to normalize positions");
}
