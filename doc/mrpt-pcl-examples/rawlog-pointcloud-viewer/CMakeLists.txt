PROJECT(rawlog-pointcloud-viewer)

cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
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
FIND_PACKAGE(MRPT REQUIRED maps obs)

# --------------------------------------------------------------------------
#  PCL libraries:
# --------------------------------------------------------------------------
find_package(PCL 1.1 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Declare the target (an executable)
ADD_EXECUTABLE(${PROJECT_NAME}
	rawlog-pcl-viewer.cpp
	)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${MRPT_LIBS} ${PCL_LIBRARIES})

# Set optimized building:
IF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
ENDIF()
