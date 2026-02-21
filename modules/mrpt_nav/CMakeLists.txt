# ------------------------------------------------------------------------------
#        Mobile Robot Programming Toolkit (MRPT)
#
# Copyright (c) 2005-2026, Jose Luis Blanco-Claraco, contributors (see Git history)
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.16)
project(mrpt_nav LANGUAGES C CXX)

find_package(mrpt_common REQUIRED)
find_package(mrpt_maps REQUIRED)
find_package(mrpt_kinematics REQUIRED)
find_package(mrpt_viz REQUIRED)

# define lib:
set(LIB_SOURCES
  src/registerAllClasses.cpp
  # holonomic/
  src/holonomic/CAbstractHolonomicReactiveMethod.cpp
  src/holonomic/CHolonomicFullEval.cpp
  src/holonomic/CHolonomicLogFileRecord.cpp
  src/holonomic/CHolonomicND.cpp
  src/holonomic/CHolonomicVFF.cpp
  src/holonomic/ClearanceDiagram.cpp
  # planners/
  src/planners/nav_plan_geometry_utils.cpp
  src/planners/PlannerRRT_common.cpp
  src/planners/PlannerRRT_SE2_TPS.cpp
  src/planners/PlannerSimple2D.cpp
  # reactive/
  src/reactive/CAbstractNavigator.cpp
  src/reactive/CAbstractPTGBasedReactive.cpp
  src/reactive/CLogFileRecord.cpp
  src/reactive/CMultiObjectiveMotionOptimizerBase.cpp
  src/reactive/CMultiObjMotionOpt_Scalarization.cpp
  src/reactive/CNavigatorManualSequence.cpp
  src/reactive/CReactiveNavigationSystem.cpp
  src/reactive/CReactiveNavigationSystem3D.cpp
  src/reactive/CRobot2NavInterface.cpp
  src/reactive/CWaypointsNavigator.cpp
  src/reactive/TCandidateMovementPTG.cpp
  src/reactive/TWaypoint.cpp
  # tpspace/
  src/tpspace/CParameterizedTrajectoryGenerator.cpp
  src/tpspace/CParameterizedTrajectoryGenerator_factory.cpp
  src/tpspace/CPTG_DiffDrive_alpha.cpp
  src/tpspace/CPTG_DiffDrive_C.cpp
  src/tpspace/CPTG_DiffDrive_CC.cpp
  src/tpspace/CPTG_DiffDrive_CCS.cpp
  src/tpspace/CPTG_DiffDrive_CollisionGridBased.cpp
  src/tpspace/CPTG_DiffDrive_CS.cpp
  src/tpspace/CPTG_Holo_Blend.cpp
  src/tpspace/CPTG_RobotShape_Circular.cpp
  src/tpspace/CPTG_RobotShape_Polygonal.cpp
  # tests
  tests/nav_plan_geometry_utils_unittest.cpp
  tests/PlannerSimple2D_unittest.cpp
  tests/PTGs_unittest.cpp
  tests/CLogFileRecord_unittest.cpp
  tests/rnav_unittest.cpp
)

set(LIB_PUBLIC_HEADERS
  include/mrpt/nav/registerAllClasses.h
  # holonomic/
  include/mrpt/nav/holonomic/CAbstractHolonomicReactiveMethod.h
  include/mrpt/nav/holonomic/CHolonomicFullEval.h
  include/mrpt/nav/holonomic/CHolonomicLogFileRecord.h
  include/mrpt/nav/holonomic/CHolonomicND.h
  include/mrpt/nav/holonomic/CHolonomicVFF.h
  include/mrpt/nav/holonomic/ClearanceDiagram.h
  # planners/
  include/mrpt/nav/planners/impl_renderMoveTree.h
  include/mrpt/nav/planners/nav_plan_geometry_utils.h
  include/mrpt/nav/planners/PlannerRRT_common.h
  include/mrpt/nav/planners/PlannerRRT_SE2_TPS.h
  include/mrpt/nav/planners/PlannerSimple2D.h
  include/mrpt/nav/planners/TMoveTree.h
  # reactive/
  include/mrpt/nav/reactive/CAbstractNavigator.h
  include/mrpt/nav/reactive/CAbstractPTGBasedReactive.h
  include/mrpt/nav/reactive/CLogFileRecord.h
  include/mrpt/nav/reactive/CMultiObjectiveMotionOptimizerBase.h
  include/mrpt/nav/reactive/CMultiObjMotionOpt_Scalarization.h
  include/mrpt/nav/reactive/CNavigatorManualSequence.h
  include/mrpt/nav/reactive/CReactiveNavigationSystem.h
  include/mrpt/nav/reactive/CReactiveNavigationSystem3D.h
  include/mrpt/nav/reactive/CRobot2NavInterface.h
  include/mrpt/nav/reactive/CRobot2NavInterfaceForSimulator.h
  include/mrpt/nav/reactive/CWaypointsNavigator.h
  include/mrpt/nav/reactive/TCandidateMovementPTG.h
  include/mrpt/nav/reactive/TWaypoint.h
  # tpspace/
  include/mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h
  include/mrpt/nav/tpspace/CPTG_DiffDrive_alpha.h
  include/mrpt/nav/tpspace/CPTG_DiffDrive_C.h
  include/mrpt/nav/tpspace/CPTG_DiffDrive_CC.h
  include/mrpt/nav/tpspace/CPTG_DiffDrive_CCS.h
  include/mrpt/nav/tpspace/CPTG_DiffDrive_CollisionGridBased.h
  include/mrpt/nav/tpspace/CPTG_DiffDrive_CS.h
  include/mrpt/nav/tpspace/CPTG_Holo_Blend.h
)

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SOURCES} ${LIB_PUBLIC_HEADERS}
  PUBLIC_LINK_LIBRARIES
    mrpt::mrpt_maps
    mrpt::mrpt_kinematics
    mrpt::mrpt_viz
  CMAKE_DEPENDENCIES
    mrpt_maps
    mrpt_kinematics
    mrpt_viz
)
