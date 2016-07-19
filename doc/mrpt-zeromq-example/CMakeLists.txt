PROJECT(mrpt_zeromq_example)

CMAKE_MINIMUM_REQUIRED(VERSION 2.4)
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
FIND_PACKAGE( MRPT REQUIRED base)

# Find ZMQ
find_path (ZMQ_INCLUDE_DIR zmq.h )
find_library (ZMQ_LIBRARY NAMES zmq )

if(ZMQ_INCLUDE)
	INCLUDE_DIRECTORIES(${ZMQ_INCLUDE})
endif()

# Set optimized building:
IF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
ENDIF()

# Subscriber:
ADD_EXECUTABLE(mrpt_zmq_example_sub main_sub.cpp)
TARGET_LINK_LIBRARIES(mrpt_zmq_example_sub ${MRPT_LIBS} ${ZMQ_LIBRARY})

# Publisher:
ADD_EXECUTABLE(mrpt_zmq_example_pub main_pub.cpp)
TARGET_LINK_LIBRARIES(mrpt_zmq_example_pub ${MRPT_LIBS} ${ZMQ_LIBRARY})
