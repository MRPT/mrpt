# ------------------------------------------------------------------------------
#        Mobile Robot Programming Toolkit (MRPT)
#
# Copyright (c) 2005-2026, Jose Luis Blanco-Claraco, contributors (see Git history)
# All rights reserved.
# Released under BSD-3 license. See LICENSE file
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.16)

# Tell CMake we'll use C++ for use in its tests/flags
project(mrpt_system LANGUAGES C CXX)

# MRPT CMake scripts: "mrpt_xxx()"
find_package(mrpt_common REQUIRED)
find_package(mrpt_core REQUIRED)
find_package(mrpt_typemeta REQUIRED)
find_package(mrpt_containers REQUIRED)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(Filesystem REQUIRED)  # Use C++17 filesystem lib

include(./cmake/find_inotify.cmake REQUIRED)     # Check for the sys/inotify headers (Linux only, in win32 we use the equivalent API for file system monitoring):

# define lib:
set(LIB_SOURCES
  src/progress.cpp
  src/CTicTac.cpp
  src/crc.cpp
  src/CObservable.cpp
  src/thread_name.cpp
  src/CControlledRateTimer.cpp
  src/hyperlink.cpp
  src/os.cpp
  src/string_utils.cpp
  src/filesystem.cpp
  src/CFileSystemWatcher.cpp
  src/md5.cpp
  src/COutputLogger.cpp
  src/memory.cpp
  src/base64.cpp
  src/CTimeLogger.cpp
  src/CRateTimer.cpp
  src/scheduler.cpp
  src/WorkerThreadsPool.cpp
  src/datetime.cpp
  src/CObserver.cpp
  src/CDirectoryExplorer.cpp
  tests/CTimeLogger_unittest.cpp
  tests/string_utils_unittest.cpp
  tests/filesystem_unittest.cpp
  tests/thread_name_unittest.cpp
  tests/datetime_unittest.cpp
  tests/base64_unittest.cpp
  tests/crc_unittest.cpp
)

set(LIB_PUBLIC_HEADERS
  include/mrpt/system/CRateTimer.h
  include/mrpt/system/CControlledRateTimer.h
  include/mrpt/system/hyperlink.h
  include/mrpt/system/CGenericMemoryPool.h
  include/mrpt/system/datetime.h
  include/mrpt/system/thread_name.h
  include/mrpt/system/filesystem.h
  include/mrpt/system/CTicTac.h
  include/mrpt/system/WorkerThreadsPool.h
  include/mrpt/system/md5.h
  include/mrpt/system/scheduler.h
  include/mrpt/system/CTimeLogger.h
  include/mrpt/system/memory.h
  include/mrpt/system/CConsoleRedirector.h
  include/mrpt/system/mrptEvent.h
  include/mrpt/system/CObservable.h
  include/mrpt/system/os.h
  include/mrpt/system/CDirectoryExplorer.h
  include/mrpt/system/progress.h
  include/mrpt/system/COutputLogger.h
  include/mrpt/system/CFileSystemWatcher.h
  include/mrpt/system/CObserver.h
  include/mrpt/system/crc.h
  include/mrpt/system/string_utils.h
)

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SOURCES} ${LIB_PUBLIC_HEADERS}
  PUBLIC_LINK_LIBRARIES
    mrpt::mrpt_core
    mrpt::mrpt_typemeta
    mrpt::mrpt_containers
#  PRIVATE_LINK_LIBRARIES
  CMAKE_DEPENDENCIES
    mrpt_core
    mrpt_typemeta
    mrpt_containers
)


target_link_libraries(${PROJECT_NAME} PRIVATE
  Threads::Threads
  CXX::Filesystem  # the C++>=17 std lib
)

# -ldl required by the plugin load mechanism loadPluginModule()
if(UNIX)
  target_link_libraries(${PROJECT_NAME}  PRIVATE ${CMAKE_DL_LIBS})
endif()


# Python wrapper:
mrpt_add_python_module(system
  python_bindings/${PROJECT_NAME}_py.cpp
)