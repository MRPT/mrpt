# ------------------------------------------------------------------------------
#        Mobile Robot Programming Toolkit (MRPT)
#
# Copyright (c) 2005-2026, Jose Luis Blanco-Claraco, contributors (see Git history)
# All rights reserved.
# Released under BSD-3 license. See LICENSE file
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.16)

# Tell CMake we'll use C++ for use in its tests/flags
project(mrpt_bayes LANGUAGES C CXX)

# MRPT CMake scripts: "mrpt_xxx()"
find_package(mrpt_common REQUIRED)
find_package(mrpt_config REQUIRED)
find_package(mrpt_math REQUIRED)


# define lib:
set(LIB_SRCS
  src/CParticleFilter.cpp
  src/CParticleFilterCapable.cpp
)

set(LIB_PUBLIC_HDRS
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
  SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
  PUBLIC_LINK_LIBRARIES
    mrpt::mrpt_config
    mrpt::mrpt_math
#  PRIVATE_LINK_LIBRARIES
  CMAKE_DEPENDENCIES
    mrpt_config
    mrpt_math
  # Other imported targets:
)

