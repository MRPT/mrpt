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

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/nav/reactive/CLogFileRecord.h>
#include <mrpt/nav/reactive/TWaypoint.h>
#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>
#include <mrpt/nav/tpspace/CPTG_DiffDrive_C.h>
#include <mrpt/nav/tpspace/CPTG_DiffDrive_CC.h>
#include <mrpt/nav/tpspace/CPTG_DiffDrive_CS.h>
#include <mrpt/nav/tpspace/CPTG_Holo_Blend.h>
#include <mrpt/nav/registerAllClasses.h>

namespace py = pybind11;
using namespace mrpt::nav;
using namespace pybind11::literals;

PYBIND11_MODULE(_bindings, m)
{
  mrpt::nav::registerAllClasses_mrpt_nav();

  // -----------------------------------------------------------------------
  // TWaypoint
  // -----------------------------------------------------------------------
  py::class_<TWaypoint>(m, "TWaypoint")
      .def(py::init<>())
      .def(
          py::init<double, double, double, bool>(), py::arg("target_x"), py::arg("target_y"),
          py::arg("allowed_distance"), py::arg("allow_skip") = true)
      .def_readwrite("target", &TWaypoint::target)
      .def_readwrite("target_heading", &TWaypoint::target_heading)
      .def_readwrite("target_frame_id", &TWaypoint::target_frame_id)
      .def_readwrite("allowed_distance", &TWaypoint::allowed_distance)
      .def_readwrite("speed_ratio", &TWaypoint::speed_ratio)
      .def_readwrite("allow_skip", &TWaypoint::allow_skip)
      .def("isValid", &TWaypoint::isValid)
      .def("getAsText", &TWaypoint::getAsText)
      .def("__repr__", &TWaypoint::getAsText);

  // -----------------------------------------------------------------------
  // TWaypointSequence
  // -----------------------------------------------------------------------
  py::class_<TWaypointSequence>(m, "TWaypointSequence")
      .def(py::init<>())
      .def_readwrite("waypoints", &TWaypointSequence::waypoints)
      .def("clear", &TWaypointSequence::clear)
      .def("getAsText", &TWaypointSequence::getAsText)
      .def("__repr__", &TWaypointSequence::getAsText)
      .def(
          "__len__",
          [](const TWaypointSequence& seq) { return seq.waypoints.size(); })
      .def(
          "__getitem__",
          [](const TWaypointSequence& seq, size_t i) -> const TWaypoint& {
            if (i >= seq.waypoints.size()) throw py::index_error();
            return seq.waypoints[i];
          },
          py::return_value_policy::reference_internal)
      .def(
          "append",
          [](TWaypointSequence& seq, const TWaypoint& wp) {
            seq.waypoints.push_back(wp);
          },
          py::arg("waypoint"));

  // -----------------------------------------------------------------------
  // TWaypointStatus
  // -----------------------------------------------------------------------
  py::class_<TWaypointStatus, TWaypoint>(m, "TWaypointStatus")
      .def(py::init<>())
      .def_readwrite("reached", &TWaypointStatus::reached)
      .def_readwrite("skipped", &TWaypointStatus::skipped);

  // -----------------------------------------------------------------------
  // CParameterizedTrajectoryGenerator (base class + factory)
  // -----------------------------------------------------------------------
  py::class_<CParameterizedTrajectoryGenerator,
             std::shared_ptr<CParameterizedTrajectoryGenerator>>(
      m, "CParameterizedTrajectoryGenerator")
      .def_static(
          "CreatePTG",
          [](const std::string& className, const std::string& iniText,
             const std::string& section, const std::string& keyPrefix) {
            mrpt::config::CConfigFileMemory cfg;
            cfg.setContent(iniText);
            return CParameterizedTrajectoryGenerator::CreatePTG(
                className, cfg, section, keyPrefix);
          },
          py::arg("ptg_class_name"), py::arg("ini_text"), py::arg("section"),
          py::arg("key_prefix") = std::string(""),
          "Factory: create a PTG from INI-format string, section, and key prefix.")
      .def("getDescription", &CParameterizedTrajectoryGenerator::getDescription)
      .def(
          "initialize",
          [](CParameterizedTrajectoryGenerator& self) { self.initialize(); })
      .def("deinitialize", &CParameterizedTrajectoryGenerator::deinitialize)
      .def("isInitialized", &CParameterizedTrajectoryGenerator::isInitialized)
      .def("getAlphaValuesCount", &CParameterizedTrajectoryGenerator::getAlphaValuesCount)
      .def("getPathCount", &CParameterizedTrajectoryGenerator::getPathCount)
      .def(
          "inverseMap_WS2TP",
          [](const CParameterizedTrajectoryGenerator& self, double x, double y,
             double tol) { return self.inverseMap_WS2TP(x, y, tol); },
          py::arg("x"), py::arg("y"), py::arg("tolerance_dist") = 0.10,
          "Map a WS point to (k, normalized_d). Returns None if no path found.")
      .def("PTG_IsIntoDomain", &CParameterizedTrajectoryGenerator::PTG_IsIntoDomain,
           py::arg("x"), py::arg("y"))
      .def(
          "loadFromConfigFile",
          [](CParameterizedTrajectoryGenerator& self, const std::string& iniText,
             const std::string& section) {
            mrpt::config::CConfigFileMemory cfg;
            cfg.setContent(iniText);
            self.loadFromConfigFile(cfg, section);
          },
          py::arg("ini_text"), py::arg("section"));

  // -----------------------------------------------------------------------
  // CLogFileRecord
  // -----------------------------------------------------------------------
  py::class_<CLogFileRecord, std::shared_ptr<CLogFileRecord>>(m, "CLogFileRecord")
      .def(py::init<>())
      .def_readwrite("nPTGs", &CLogFileRecord::nPTGs)
      .def_readwrite("robotPoseLocalization", &CLogFileRecord::robotPoseLocalization)
      .def_readwrite("robotPoseOdometry", &CLogFileRecord::robotPoseOdometry)
      .def_readwrite("relPoseSense", &CLogFileRecord::relPoseSense)
      .def_readwrite("relPoseVelCmd", &CLogFileRecord::relPoseVelCmd)
      .def_readwrite("WS_targets_relative", &CLogFileRecord::WS_targets_relative)
      .def_readwrite("infoPerPTG", &CLogFileRecord::infoPerPTG)
      .def(
          "__repr__",
          [](const CLogFileRecord& r) {
            return std::string("<CLogFileRecord nPTGs=") + std::to_string(r.nPTGs) + ">";
          });
}
