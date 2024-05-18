project(rawlog-pointcloud-viewer)

cmake_minimum_required(VERSION 3.16)

# --------------------------------------------------------------------------
#   The list of "libs" which can be included can be found in:
#     https://www.mrpt.org/Libraries
#
#   The dependencies of a library are automatically added, so you only
#    need to specify the top-most libraries your code depend on.
# --------------------------------------------------------------------------
find_package(MRPT REQUIRED maps obs)

# --------------------------------------------------------------------------
#  PCL libraries:
# --------------------------------------------------------------------------
find_package(PCL 1.1 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Declare the target (an executable)
add_executable(${PROJECT_NAME}
	rawlog-pcl-viewer.cpp
	)
target_link_libraries(${PROJECT_NAME} ${MRPT_LIBS} ${PCL_LIBRARIES})

# Set optimized building:
if(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
endif()
