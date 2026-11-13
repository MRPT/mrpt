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

#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/kinematics/CVehicleVelCmd.h>
#include <mrpt/math/TTwist2D.h>
#include <mrpt/nav/reactive/CLogFileRecord.h>
#include <mrpt/nav/reactive/TWaypoint.h>
#include <mrpt/nav/registerAllClasses.h>
#include <mrpt/nav/tpspace/CPTG_DiffDrive_C.h>
#include <mrpt/nav/tpspace/CPTG_DiffDrive_CC.h>
#include <mrpt/nav/tpspace/CPTG_DiffDrive_CS.h>
#include <mrpt/nav/tpspace/CPTG_Holo_Blend.h>
#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <optional>

namespace py = pybind11;
using namespace mrpt::nav;
using namespace pybind11::literals;

PYBIND11_MODULE(_bindings, m)
{
  mrpt::nav::registerAllClasses_mrpt_nav();

  // -----------------------------------------------------------------------
  // TWaypoint
  // -----------------------------------------------------------------------
  py::class_<TWaypoint>(m, "TWaypoint", "A single navigation waypoint within a TWaypointSequence.")
      .def(py::init<>(), "Ctor with default values.")
      .def(
          py::init<double, double, double, bool>(), py::arg("target_x"), py::arg("target_y"),
          py::arg("allowed_distance"), py::arg("allow_skip") = true,
          "Builds a waypoint from its target (x, y), allowed distance and whether it can be "
          "skipped.")
      .def_readwrite("target", &TWaypoint::target)
      .def_readwrite("target_heading", &TWaypoint::target_heading)
      .def_readwrite("target_frame_id", &TWaypoint::target_frame_id)
      .def_readwrite("allowed_distance", &TWaypoint::allowed_distance)
      .def_readwrite("speed_ratio", &TWaypoint::speed_ratio)
      .def_readwrite("allow_skip", &TWaypoint::allow_skip)
      .def(
          "isValid", &TWaypoint::isValid,
          "Check whether all the minimum mandatory fields have been filled by the user.")
      .def("getAsText", &TWaypoint::getAsText, "Get in human-readable format.")
      .def("__repr__", &TWaypoint::getAsText);

  // -----------------------------------------------------------------------
  // TWaypointSequence
  // -----------------------------------------------------------------------
  py::class_<TWaypointSequence>(
      m, "TWaypointSequence", "A sequence of waypoints for the waypoints navigator.")
      .def(py::init<>(), "Ctor with default values.")
      .def_readwrite("waypoints", &TWaypointSequence::waypoints)
      .def("clear", &TWaypointSequence::clear, "Removes all waypoints.")
      .def(
          "getAsText", &TWaypointSequence::getAsText,
          "Gets navigation params as a human-readable format.")
      .def("__repr__", &TWaypointSequence::getAsText)
      .def("__len__", [](const TWaypointSequence& seq) { return seq.waypoints.size(); })
      .def(
          "__getitem__",
          [](const TWaypointSequence& seq, size_t i) -> const TWaypoint&
          {
            if (i >= seq.waypoints.size()) throw py::index_error();
            return seq.waypoints[i];
          },
          py::return_value_policy::reference_internal)
      .def(
          "append",
          [](TWaypointSequence& seq, const TWaypoint& wp) { seq.waypoints.push_back(wp); },
          py::arg("waypoint"), "Appends a waypoint to the sequence.");

  // -----------------------------------------------------------------------
  // TWaypointStatus
  // -----------------------------------------------------------------------
  py::class_<TWaypointStatus, TWaypoint>(
      m, "TWaypointStatus", "A TWaypoint augmented with runtime execution status fields.")
      .def(py::init<>(), "Default constructor.")
      .def_readwrite("reached", &TWaypointStatus::reached)
      .def_readwrite("skipped", &TWaypointStatus::skipped);

  // -----------------------------------------------------------------------
  // CParameterizedTrajectoryGenerator (base class + factory)
  // -----------------------------------------------------------------------
  py::class_<CParameterizedTrajectoryGenerator, std::shared_ptr<CParameterizedTrajectoryGenerator>>(
      m, "CParameterizedTrajectoryGenerator",
      "Base class for all Parameterized Trajectory Generators (PTGs).")
      .def_static(
          "CreatePTG",
          [](const std::string& className, const std::string& iniText, const std::string& section,
             const std::string& keyPrefix)
          {
            mrpt::config::CConfigFileMemory cfg;
            cfg.setContent(iniText);
            return CParameterizedTrajectoryGenerator::CreatePTG(className, cfg, section, keyPrefix);
          },
          py::arg("ptg_class_name"), py::arg("ini_text"), py::arg("section"),
          py::arg("key_prefix") = std::string(""),
          "Factory: create a PTG from INI-format string, section, and key prefix.")
      .def(
          "getDescription", &CParameterizedTrajectoryGenerator::getDescription,
          "Gets a short textual description of the PTG and its parameters.")
      .def(
          "initialize",
          [](CParameterizedTrajectoryGenerator& self, const std::string& cacheFilename,
             bool verbose) { self.initialize(cacheFilename, verbose); },
          "cacheFilename"_a = std::string(), "verbose"_a = false,
          "Initializes the PTG; call after setting all its parameters and before using it. PTGs "
          "based on collision grids (CPTG_DiffDrive_*) load their grid from cacheFilename, or "
          "compute and save it there; an empty name means \"cache_<description>.bin.gz\" in the "
          "current directory. Other PTGs ignore it.")
      .def(
          "deinitialize", &CParameterizedTrajectoryGenerator::deinitialize,
          "De-initializes the PTG, so its parameters can be changed.")
      .def(
          "isInitialized", &CParameterizedTrajectoryGenerator::isInitialized,
          "Returns true if initialize() has been called and there was no errors, so the PTG is "
          "ready to be queried for paths, obstacles, etc.")
      .def(
          "getAlphaValuesCount", &CParameterizedTrajectoryGenerator::getAlphaValuesCount,
          "Get the number of different, discrete paths in this family.")
      .def(
          "getPathCount", &CParameterizedTrajectoryGenerator::getPathCount,
          "Get the number of different, discrete paths in this family.")
      .def(
          "inverseMap_WS2TP",
          [](const CParameterizedTrajectoryGenerator& self, double x, double y, double tol)
          { return self.inverseMap_WS2TP(x, y, tol); },
          py::arg("x"), py::arg("y"), py::arg("tolerance_dist") = 0.10,
          "Map a WS point to (k, normalized_d). Returns None if no path found.")
      .def(
          "PTG_IsIntoDomain", &CParameterizedTrajectoryGenerator::PTG_IsIntoDomain, py::arg("x"),
          py::arg("y"), "Returns true if (x, y) is within the PTG domain.")
      .def(
          "loadFromConfigFile",
          [](CParameterizedTrajectoryGenerator& self, const std::string& iniText,
             const std::string& section)
          {
            mrpt::config::CConfigFileMemory cfg;
            cfg.setContent(iniText);
            self.loadFromConfigFile(cfg, section);
          },
          py::arg("ini_text"), py::arg("section"),
          "Loads the PTG parameters from a configuration file section.")
      // Paths
      .def(
          "getPathStepCount", &CParameterizedTrajectoryGenerator::getPathStepCount, "k"_a,
          "Number of discrete steps of path k.")
      .def(
          "getPathPose", &CParameterizedTrajectoryGenerator::getPathPose, "k"_a, "step"_a,
          "Pose of the vehicle (TPose2D) at a step of path k, relative to its start.")
      .def(
          "getPathTwist", &CParameterizedTrajectoryGenerator::getPathTwist, "k"_a, "step"_a,
          "Velocity (TTwist2D) at a step of path k, in the frame of the start of the path.")
      .def(
          "getPathDist", &CParameterizedTrajectoryGenerator::getPathDist, "k"_a, "step"_a,
          "Distance traveled along path k at a step, in pseudometers (not normalized).")
      .def(
          "getPathStepForDist",
          [](const CParameterizedTrajectoryGenerator& self, uint16_t k,
             double dist) -> std::optional<uint32_t> { return self.getPathStepForDist(k, dist); },
          "k"_a, "dist"_a,
          "Largest step of path k with a traveled distance < dist (pseudometers), or None if "
          "there is none.")
      .def(
          "getPathStepDuration", &CParameterizedTrajectoryGenerator::getPathStepDuration,
          "Duration of each path step, in seconds.")
      .def(
          "getMaxLinVel", &CParameterizedTrajectoryGenerator::getMaxLinVel,
          "Maximum linear velocity of this PTG (m/s).")
      .def(
          "getMaxAngVel", &CParameterizedTrajectoryGenerator::getMaxAngVel,
          "Maximum angular velocity of this PTG (rad/s).")
      .def(
          "index2alpha",
          [](const CParameterizedTrajectoryGenerator& self, uint16_t k)
          { return self.index2alpha(k); },
          "k"_a, "Direction alpha (radians, in [-pi, pi]) of the discrete path index k.")
      .def(
          "alpha2index",
          [](const CParameterizedTrajectoryGenerator& self, double alpha)
          { return self.alpha2index(alpha); },
          "alpha"_a, "Discrete path index for the direction alpha (radians).")
      .def(
          "getRefDistance", &CParameterizedTrajectoryGenerator::getRefDistance,
          "Reference distance (meters): the length of the paths, used to normalize TP-Space "
          "distances.")
      .def(
          "setRefDistance", &CParameterizedTrajectoryGenerator::setRefDistance, "refDist"_a,
          "Sets the reference distance (meters).")
      .def(
          "directionToMotionCommand",
          [](const CParameterizedTrajectoryGenerator& self, uint16_t k)
          {
            if (k >= self.getAlphaValuesCount())
            {
              throw py::index_error("path index out of range");
            }
            return self.directionToMotionCommand(k);
          },
          "k"_a, "Velocity command (a mrpt.kinematics.CVehicleVelCmd subclass) to follow path k.")
      // Obstacles in TP-Space
      .def(
          "initTPObstacles",
          [](const CParameterizedTrajectoryGenerator& self)
          {
            std::vector<double> tp;
            self.initTPObstacles(tp);
            return tp;
          },
          "Returns the initial TP-Obstacles, one per path: the collision-free distance of each "
          "path without obstacles (pseudometers).")
      .def(
          "initTPObstacleSingle",
          [](const CParameterizedTrajectoryGenerator& self, uint16_t k)
          {
            double tp = 0;
            self.initTPObstacleSingle(k, tp);
            return tp;
          },
          "k"_a, "Returns the initial TP-Obstacle of path k, as initTPObstacles().")
      .def(
          "updateTPObstacle",
          [](const CParameterizedTrajectoryGenerator& self, double ox, double oy,
             std::vector<double> tp_obstacles)
          {
            if (tp_obstacles.size() != self.getAlphaValuesCount())
            {
              throw py::value_error("tp_obstacles must have getAlphaValuesCount() elements");
            }
            self.updateTPObstacle(ox, oy, tp_obstacles);
            return tp_obstacles;
          },
          "ox"_a, "oy"_a, "tp_obstacles"_a,
          "Returns tp_obstacles (from initTPObstacles()) updated with an obstacle point (ox, oy), "
          "relative to the origin of the PTG: the collision-free distance of each path.")
      .def(
          "updateTPObstacleSingle",
          [](const CParameterizedTrajectoryGenerator& self, double ox, double oy, uint16_t k,
             double tp_obstacle_k)
          {
            if (k >= self.getAlphaValuesCount())
            {
              throw py::index_error("path index out of range");
            }
            self.updateTPObstacleSingle(ox, oy, k, tp_obstacle_k);
            return tp_obstacle_k;
          },
          "ox"_a, "oy"_a, "k"_a, "tp_obstacle_k"_a,
          "Like updateTPObstacle(), for path k only: returns tp_obstacle_k (from "
          "initTPObstacleSingle()) updated with the obstacle point (ox, oy).");

  // -----------------------------------------------------------------------
  // CLogFileRecord
  // -----------------------------------------------------------------------
  py::class_<CLogFileRecord, std::shared_ptr<CLogFileRecord>> logFileRecord(
      m, "CLogFileRecord", "One navigation step of the reactive navigator log.");

  py::class_<CLogFileRecord::TInfoPerPTG>(
      logFileRecord, "TInfoPerPTG", "Log data of one PTG in one navigation step.")
      .def(py::init<>(), "Default constructor.")
      .def_readwrite(
          "PTG_desc", &CLogFileRecord::TInfoPerPTG::PTG_desc, "Short description of the PTG")
      .def_property(
          "TP_Obstacles",
          [](const CLogFileRecord::TInfoPerPTG& i)
          { return std::vector<float>(i.TP_Obstacles.begin(), i.TP_Obstacles.end()); },
          [](CLogFileRecord::TInfoPerPTG& i, const std::vector<float>& v)
          {
            i.TP_Obstacles.resize(v.size());
            std::copy(v.begin(), v.end(), i.TP_Obstacles.begin());
          },
          "Distance to obstacles in TP-Space (pseudometers), for directions from -pi to pi")
      .def_readwrite(
          "TP_Targets", &CLogFileRecord::TInfoPerPTG::TP_Targets, "Target(s) in TP-Space")
      .def_readwrite(
          "TP_Robot", &CLogFileRecord::TInfoPerPTG::TP_Robot,
          "Robot location in TP-Space (normally the origin)")
      .def_readwrite(
          "timeForTPObsTransformation", &CLogFileRecord::TInfoPerPTG::timeForTPObsTransformation,
          "Time to transform obstacles into TP-Space [s]")
      .def_readwrite(
          "timeForHolonomicMethod", &CLogFileRecord::TInfoPerPTG::timeForHolonomicMethod,
          "Time spent in the holonomic method [s]")
      .def_readwrite(
          "desiredDirection", &CLogFileRecord::TInfoPerPTG::desiredDirection,
          "Direction chosen by the holonomic method [rad]")
      .def_readwrite(
          "desiredSpeed", &CLogFileRecord::TInfoPerPTG::desiredSpeed,
          "Speed chosen by the holonomic method")
      .def_readwrite(
          "evaluation", &CLogFileRecord::TInfoPerPTG::evaluation, "Final score of this candidate")
      .def_readwrite(
          "evalFactors", &CLogFileRecord::TInfoPerPTG::evalFactors, "Evaluation factors, by name")
      .def(
          "__repr__", [](const CLogFileRecord::TInfoPerPTG& i)
          { return "<TInfoPerPTG '" + i.PTG_desc + "'>"; });

  logFileRecord.def(py::init<>(), "Constructor, builds an empty record.")
      .def_readwrite("nPTGs", &CLogFileRecord::nPTGs)
      .def_readwrite("robotPoseLocalization", &CLogFileRecord::robotPoseLocalization)
      .def_readwrite("robotPoseOdometry", &CLogFileRecord::robotPoseOdometry)
      .def_readwrite("relPoseSense", &CLogFileRecord::relPoseSense)
      .def_readwrite("relPoseVelCmd", &CLogFileRecord::relPoseVelCmd)
      .def_readwrite("WS_targets_relative", &CLogFileRecord::WS_targets_relative)
      .def_readwrite("infoPerPTG", &CLogFileRecord::infoPerPTG)
      .def(
          "__repr__", [](const CLogFileRecord& r)
          { return std::string("<CLogFileRecord nPTGs=") + std::to_string(r.nPTGs) + ">"; });
}
