PROJECT(mrpt_zeromq_example)

CMAKE_MINIMUM_REQUIRED(VERSION 2.4)
if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)  # Required by CMake 2.7+
endif(COMMAND cmake_policy)

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
endif(ZMQ_INCLUDE)

# Declare the target (an executable)
ADD_EXECUTABLE(mrpt_zeromq_example
	main.cpp
	)
TARGET_LINK_LIBRARIES(mrpt_zeromq_example ${MRPT_LIBS} ${ZMQ_LIBRARY})

# Set optimized building:
IF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
ENDIF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")

