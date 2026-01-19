# ------------------------------------------------------------------------------
#        Mobile Robot Programming Toolkit (MRPT)
#
# Copyright (c) 2005-2026, Jose Luis Blanco-Claraco, contributors (see Git history)
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# ------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

# Tell CMake we'll use C++ for use in its tests/flags
project(mrpt_kinematics LANGUAGES C CXX)

# MRPT CMake scripts: "mrpt_xxx()"
find_package(mrpt_common REQUIRED)
find_package(mrpt_viz REQUIRED)

# define lib:
set(LIB_SOURCES
  src/CVehicleSimulVirtualBase.cpp
  src/CVehicleVelCmd_DiffDriven.cpp
  src/CVehicleVelCmd.cpp
  src/CVehicleVelCmd_Holo.cpp
  src/CVehicleSimul_Holo.cpp
  src/registerAllClasses.cpp
  src/CKinematicChain.cpp
  src/CVehicleSimul_DiffDriven.cpp
)

set(LIB_PUBLIC_HEADERS
  include/mrpt/kinematics/CVehicleSimul_Holo.h
  include/mrpt/kinematics/CVehicleSimul_DiffDriven.h
  include/mrpt/kinematics/registerAllClasses.h
  include/mrpt/kinematics/CVehicleVelCmd_Holo.h
  include/mrpt/kinematics/CVehicleVelCmd.h
  include/mrpt/kinematics/CVehicleVelCmd_DiffDriven.h
  include/mrpt/kinematics/CVehicleSimulVirtualBase.h
  include/mrpt/kinematics/CKinematicChain.h
  include/mrpt/kinematics.h
)

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SOURCES} ${LIB_PUBLIC_HEADERS}
  PUBLIC_LINK_LIBRARIES
    mrpt::mrpt_viz
#  PRIVATE_LINK_LIBRARIES
  CMAKE_DEPENDENCIES
    mrpt_viz
)

# Python wrapper:
#mrpt_add_python_module(graphs
#  python_bindings/${PROJECT_NAME}_py.cpp
#)