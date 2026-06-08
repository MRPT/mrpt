# ------------------------------------------------------------------------------
#        Mobile Robot Programming Toolkit (MRPT)
#
# Copyright (c) 2005-2026, Jose Luis Blanco-Claraco, contributors (see Git history)
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# ------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

# Tell CMake we'll use C++ for use in its tests/flags
project(mrpt_graphs LANGUAGES C CXX)

# MRPT CMake scripts: "mrpt_xxx()"
find_package(mrpt_common REQUIRED)
find_package(mrpt_poses REQUIRED)
find_package(mrpt_io REQUIRED)
find_package(mrpt_viz REQUIRED)
find_package(Eigen3 REQUIRED)

# define lib:
set(LIB_SOURCES
  src/ScalarFactorGraph.cpp
  src/registerAllClasses.cpp
  src/CGraphPartitioner.cpp
)

set(LIB_UNIT_TEST_SOURCES
  tests/ScalarFactorGraph_unittest.cpp
)

set(LIB_PUBLIC_HEADERS
  include/mrpt/graphs/CAStarAlgorithm.h
  include/mrpt/graphs/CGraphPartitioner.h
  include/mrpt/graphs/dijkstra.h
  include/mrpt/graphs/registerAllClasses.h
  include/mrpt/graphs/ScalarFactorGraph.h
  include/mrpt/graphs/CNetworkOfPoses_impl.h
  include/mrpt/graphs/TNodeAnnotations.h
  include/mrpt/graphs/CDirectedGraph.h
  include/mrpt/graphs/TNodeID.h
  include/mrpt/graphs/CDirectedTree.h
  include/mrpt/graphs/CNetworkOfPoses.h
  include/mrpt/graphs.h
)

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SOURCES} ${LIB_PUBLIC_HEADERS}
  UNIT_TEST_SOURCES ${LIB_UNIT_TEST_SOURCES}
  PUBLIC_LINK_LIBRARIES
    mrpt::mrpt_poses
    mrpt::mrpt_io
    mrpt::mrpt_viz
  PRIVATE_LINK_LIBRARIES
    Eigen3::Eigen
  CMAKE_DEPENDENCIES
    mrpt_poses
    mrpt_io
    mrpt_viz
  UNITTEST_LINK_LIBRARIES
    Eigen3::Eigen
)

# Python wrapper:
mrpt_add_python_module(graphs
  python_bindings/${PROJECT_NAME}_py.cpp
)

mrpt_add_python_binding_test(graphs
  SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/tests/python_graphs_test.py
)
