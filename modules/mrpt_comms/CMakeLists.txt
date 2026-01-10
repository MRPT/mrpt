# ------------------------------------------------------------------------------
#        Mobile Robot Programming Toolkit (MRPT)
#
# Copyright (c) 2005-2026, Jose Luis Blanco-Claraco, contributors (see Git history)
# All rights reserved.
# Released under BSD-3 license. See LICENSE file
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.16)

# Tell CMake we'll use C++ for use in its tests/flags
project(mrpt_comms LANGUAGES C CXX)

# MRPT CMake scripts: "mrpt_xxx()"
find_package(mrpt_common REQUIRED)
find_package(mrpt_io REQUIRED)

if (BUILD_TESTING)
  find_package(mrpt_poses REQUIRED) # for unit tests
endif()

include(./cmake/script_ftdi.cmake REQUIRED)


# define lib:
set(LIB_SRCS
  src/CClientTCPSocket.cpp
  src/CInterfaceFTDI_common.cpp
  src/CInterfaceFTDI_LIN.cpp
  src/CInterfaceFTDI_WIN.cpp
  src/CSerialPort.cpp
  src/CServerTCPSocket_common.cpp
  src/CServerTCPSocket.cpp
  src/net_utils.cpp
  src/nodelets.cpp
  # tests ----
  tests/nodelets_unittest.cpp
  tests/sockets_unittest.cpp
)

set(LIB_PUBLIC_HDRS
  include/mrpt/comms.h
  include/mrpt/comms/CServerTCPSocket.h
  include/mrpt/comms/nodelets.h
  include/mrpt/comms/registerAllClasses.h
  include/mrpt/comms/CSerialPort.h
  include/mrpt/comms/CInterfaceFTDI.h
  include/mrpt/comms/CClientTCPSocket.h
  include/mrpt/comms/net_utils.h
)

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
  PUBLIC_LINK_LIBRARIES
    mrpt::mrpt_io
  CMAKE_DEPENDENCIES
    mrpt_io
  UNITTEST_LINK_LIBRARIES
    mrpt::mrpt_poses
)

target_link_libraries(${PROJECT_NAME} PRIVATE Threads::Threads)

if(CMAKE_MRPT_HAS_FTDI_SYSTEM)
  target_link_libraries(${PROJECT_NAME} PRIVATE imp_ftdi)
endif()
if (MINGW)
  target_link_libraries(${PROJECT_NAME} PRIVATE WS2_32)
endif()
