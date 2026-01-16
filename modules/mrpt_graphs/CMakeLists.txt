# ------------------------------------------------------------------------------
#        Mobile Robot Programming Toolkit (MRPT)
#
# Copyright (c) 2005-2026, Jose Luis Blanco-Claraco, contributors (see Git history)
# All rights reserved.
# Released under BSD-3 license. See LICENSE file
# ------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

# Tell CMake we'll use C++ for use in its tests/flags
project(mrpt_graphs LANGUAGES C CXX)

# MRPT CMake scripts: "mrpt_xxx()"
find_package(mrpt_common REQUIRED)
find_package(mrpt_poses REQUIRED)

# define lib:
set(LIB_SOURCES
  src/ScalarFactorGraph.cpp
  src/registerAllClasses.cpp
  src/CGraphPartitioner.cpp
  # tests ----
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
  PUBLIC_LINK_LIBRARIES
    mrpt::mrpt_poses
#  PRIVATE_LINK_LIBRARIES
  CMAKE_DEPENDENCIES
    mrpt_poses
)

# Python wrapper:
#mrpt_add_python_module(graphs
#  python_bindings/${PROJECT_NAME}_py.cpp
#)