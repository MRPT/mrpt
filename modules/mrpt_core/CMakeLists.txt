# ------------------------------------------------------------------------------
#        Mobile Robot Programming Toolkit (MRPT)
#
# Copyright (c) 2005-2025, Jose Luis Blanco-Claraco, contributors (see Git history)
# All rights reserved.
# Released under BSD-3 license. See LICENSE file
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.16)

# Tell CMake we'll use C++ for use in its tests/flags
project(mrpt_core LANGUAGES C CXX)

# MRPT CMake scripts: "mrpt_xxx()"
find_package(mrpt_common REQUIRED)

# find dependencies:
find_package(Threads REQUIRED)

include(./cmake/bfd.cmake REQUIRED)
include(./cmake/detect_functions_headers.cmake REQUIRED)
include(./cmake/detect_word_size.cmake REQUIRED)
include(./cmake/exceptions_and_backtraces.cmake REQUIRED)
include(./cmake/max_alignment.cmake REQUIRED)
include(./cmake/simd.cmake REQUIRED)
include(./cmake/tbb.cmake REQUIRED)

include(./cmake/script_create_version_h.cmake REQUIRED)

set(MRPT_WITH_MATLAB_WRAPPER OFF CACHE BOOL "Build with compatibility options for MEX wrapper?.")
include(./cmake/script_matlab.cmake REQUIRED)      # Support for Matlab MEX functions generation
#if(MRPT_WITH_MATLAB_WRAPPER)
#    add_subdirectory(mex/apps)# The MEX applications
#endif()


# define lib:
set(LIB_SRCS
  src/constexpr_for_unittest.cpp
  src/cpu_unittest.cpp
  src/bits_math_unittest.cpp
  src/reverse_bytes_unittest.cpp
  src/Clock.cpp
  src/format_unittest.cpp
  src/demangle.cpp
  src/format.cpp
  src/exceptions.cpp
  src/WorkerThreadsPool_unittest.cpp
  src/cpu.cpp
  src/exception_unittest.cpp
  src/reverse_bytes.cpp
  src/backtrace.cpp
  src/aligned_malloc_unittest.cpp
  src/Clock_unittest.cpp
  src/lock_helper_unittest.cpp
  src/from_string_unittest.cpp
  src/aligned_malloc.cpp
  src/WorkerThreadsPool.cpp
  src/is_defined_unittest.cpp
  src/round_unittest.cpp
  src/winerror2str.cpp
)

set(LIB_PUBLIC_HDRS
  include/mrpt/3rdparty/spimpl.h
  include/mrpt/registerAllClasses.h
  include/mrpt/core/optional_ref.h
  include/mrpt/core/backtrace.h
  include/mrpt/core/alignment_req.h
  include/mrpt/core/aligned_std_vector.h
  include/mrpt/core/round.h
  include/mrpt/core/cpu.h
  include/mrpt/core/WorkerThreadsPool.h
  include/mrpt/core/Clock.h
  include/mrpt/core/bits_mem.h
  include/mrpt/core/lock_helper.h
  include/mrpt/core/SSE_types.h
  include/mrpt/core/format.h
  include/mrpt/core/safe_pointers.h
  include/mrpt/core/bits_math.h
  include/mrpt/core/exceptions.h
  include/mrpt/core/constexpr_for.h
  include/mrpt/core/reverse_bytes.h
  include/mrpt/core/from_string.h
  include/mrpt/core/aligned_allocator.h
  include/mrpt/core/is_defined.h
  include/mrpt/core/abs_diff.h
  include/mrpt/core/winerror2str.h
  include/mrpt/core/demangle.h
  include/mrpt/core/is_shared_ptr.h
  include/mrpt/core/aligned_std_basicstring.h
  include/mrpt/core/integer_select.h
  include/mrpt/core/bit_cast.h
  include/mrpt/core/pimpl.h
  include/mrpt/core/get_env.h
  include/mrpt/core/Stringifyable.h
  include/mrpt/core/common.h
  include/mrpt/core/initializer.h
  include/mrpt/core/SSE_macros.h
  include/mrpt/core/byte_manip.h
)

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
#  PUBLIC_LINK_LIBRARIES
#   xxx
#  PRIVATE_LINK_LIBRARIES
#  CMAKE_DEPENDENCIES
#   xxx
)

target_link_libraries(${PROJECT_NAME} PRIVATE Threads::Threads) # for WorkerThreads

# Fix a MSVC binary-breaking compatibility in MSVC 2017 15.8:
if (MSVC)
  target_compile_definitions(${PROJECT_NAME} PUBLIC -D_ENABLE_EXTENDED_ALIGNED_STORAGE)
endif()

if(WIN32)
  target_link_libraries(${PROJECT_NAME} PRIVATE Dbghelp) # For mrpt::demangle(), mrpt::callStackBackTrace()
else()
  target_link_libraries(${PROJECT_NAME} PRIVATE ${CMAKE_DL_LIBS}) # For mrpt::callStackBackTrace()
endif()

if (CMAKE_MRPT_HAS_BFD)
  target_link_libraries(${PROJECT_NAME} PRIVATE ${BFD_LIBRARIES})
endif()

target_compile_definitions(${PROJECT_NAME} PRIVATE MRPT_EXCEPTIONS_CALL_STACK_MAX_DEPTH=${MRPT_EXCEPTIONS_CALL_STACK_MAX_DEPTH}U)
if (MRPT_EXCEPTIONS_WITH_CALL_STACK)
  target_compile_definitions(${PROJECT_NAME} PRIVATE MRPT_EXCEPTIONS_WITH_CALL_STACK)
endif()