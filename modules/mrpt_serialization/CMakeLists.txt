# ------------------------------------------------------------------------------
#        Mobile Robot Programming Toolkit (MRPT)
#
# Copyright (c) 2005-2025, Jose Luis Blanco-Claraco, contributors (see Git history)
# All rights reserved.
# Released under BSD-3 license. See LICENSE file
# ------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

# Tell CMake we'll use C++ for use in its tests/flags
project(mrpt_serialization LANGUAGES C CXX)

# MRPT CMake scripts: "mrpt_xxx()"
find_package(mrpt_common REQUIRED)
find_package(mrpt_rtti REQUIRED)

include(./cmake/script_jsoncpp.cmake REQUIRED)     # Check for jsoncpp


# define lib:
set(LIB_SRCS
  src/CSerializable.cpp
  src/CArchive.cpp
  src/CMessage.cpp
  src/CSchemeArchiveBase.cpp
  src/CSchemeArchive.cpp
)

set(LIB_PUBLIC_HDRS
  include/mrpt/serialization/metaprogramming_serialization.h
  include/mrpt/serialization/optional_serialization.h
  include/mrpt/serialization/registerAllClasses.h
  include/mrpt/serialization/archiveFrom_std_streams.h
  include/mrpt/serialization/aligned_serialization.h
  include/mrpt/serialization/serialization_frwds.h
  include/mrpt/serialization/CArchive.h
  include/mrpt/serialization/zmq_serialization.h
  include/mrpt/serialization/bimap_serialization.h
  include/mrpt/serialization/CSerializable.h
  include/mrpt/serialization/archiveFrom_std_vector.h
  include/mrpt/serialization/CSchemeArchiveBase.h
  include/mrpt/serialization/CSchemeArchive.h
  include/mrpt/serialization/stl_serialization.h
  include/mrpt/serialization/CMessage.h
)

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
  PUBLIC_LINK_LIBRARIES
    mrpt::mrpt_rtti
#  PRIVATE_LINK_LIBRARIES
  CMAKE_DEPENDENCIES
    mrpt_rtti
)

# Extra dependencies required by unit tests in this module:
# Include jsoncpp only if present (optional)
if (CMAKE_MRPT_HAS_JSONCPP)
	if(TARGET "jsoncpp_lib")
		set(tst_json_dep "jsoncpp_lib")
	else()
		set(tst_json_dep "jsoncpp_lib_static")
	endif()
else()
	set(tst_json_dep "")
endif()

# NOTE: unit tests for this module are in other modules downstream, like "mrpt_io" or "mrpt_poses".

if (tst_json_dep)
  target_link_libraries(${PROJECT_NAME} PRIVATE ${tst_json_dep})

  if (TARGET test_mrpt_serialization)
    target_link_libraries(test_mrpt_serialization ${tst_json_dep})
  endif()
endif()



