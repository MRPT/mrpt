project(mrpt_zeromq_example)

cmake_minimum_required(VERSION 2.4)
if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)  # Required by CMake 2.7+
endif()

# --------------------------------------------------------------------------
#   The list of "libs" which can be included can be found in:
#     http://www.mrpt.org/Libraries
#
#   The dependencies of a library are automatically added, so you only
#    need to specify the top-most libraries your code depend on.
# --------------------------------------------------------------------------
find_package( MRPT REQUIRED base)

# Find ZMQ
find_path (ZMQ_INCLUDE_DIR zmq.h )
find_library (ZMQ_LIBRARY NAMES zmq )

if(ZMQ_INCLUDE)
	include_directories(${ZMQ_INCLUDE})
endif()

# Set optimized building:
if(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
endif()

# Subscriber:
add_executable(mrpt_zmq_example_sub main_sub.cpp)
target_link_libraries(mrpt_zmq_example_sub ${MRPT_LIBS} ${ZMQ_LIBRARY})

# Publisher:
add_executable(mrpt_zmq_example_pub main_pub.cpp)
target_link_libraries(mrpt_zmq_example_pub ${MRPT_LIBS} ${ZMQ_LIBRARY})
