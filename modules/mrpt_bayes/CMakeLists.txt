# ------------------------------------------------------------------------------
#        Mobile Robot Programming Toolkit (MRPT)
#
# Copyright (c) 2005-2026, Jose Luis Blanco-Claraco, contributors (see Git history)
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.16)

# Tell CMake we'll use C++ for use in its tests/flags
project(mrpt_bayes LANGUAGES C CXX)

# MRPT CMake scripts: "mrpt_xxx()"
find_package(mrpt_common REQUIRED)
find_package(mrpt_config REQUIRED)
find_package(mrpt_math REQUIRED)
find_package(mrpt_random REQUIRED)
find_package(Eigen3 REQUIRED)


# define lib:
set(LIB_SOURCES
  src/CParticleFilter.cpp
  src/CParticleFilterCapable.cpp
)

set(LIB_UNIT_TEST_SOURCES
  tests/bayes_unittest.cpp
)

set(LIB_PUBLIC_HEADERS
  include/mrpt/bayes/CRejectionSamplingCapable.h
  include/mrpt/bayes/CKalmanFilterCapable.h
  include/mrpt/bayes/registerAllClasses.h
  include/mrpt/bayes/CParticleFilterCapable.h
  include/mrpt/bayes/CProbabilityParticle.h
  include/mrpt/bayes/CParticleFilterData.h
  include/mrpt/bayes/CKalmanFilterCapable_impl.h
  include/mrpt/bayes/CParticleFilter.h
)

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SOURCES} ${LIB_PUBLIC_HEADERS}
  UNIT_TEST_SOURCES ${LIB_UNIT_TEST_SOURCES}
  PUBLIC_LINK_LIBRARIES
    mrpt::mrpt_config
    mrpt::mrpt_math
    mrpt::mrpt_random
  PRIVATE_LINK_LIBRARIES
    Eigen3::Eigen
  CMAKE_DEPENDENCIES
    mrpt_config
    mrpt_math
    mrpt_random
  UNITTEST_LINK_LIBRARIES
    mrpt::mrpt_random
)

mrpt_add_python_module(bayes
  python_bindings/${PROJECT_NAME}_py.cpp
)


mrpt_add_python_binding_test(bayes
  SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/tests/python_bayes_test.py
)

