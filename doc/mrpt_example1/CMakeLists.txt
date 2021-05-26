project(mrpt_example1)

cmake_minimum_required(VERSION 3.8)

# --------------------------------------------------------------------------
#   The list of "libs" which can be included can be found in:
#     https://www.mrpt.org/Libraries
#
#   The dependencies of a library are automatically added, so you only
#    need to specify the top-most libraries your code depend on.
# --------------------------------------------------------------------------
find_package(mrpt-slam REQUIRED)
message(STATUS "mrpt-poses_VERSION: ${mrpt-poses_VERSION}")
# or:
#find_package(MRPT COMPONENTS poses) # OPTIONAL_COMPONENTS math)
#message(STATUS "MRPT_VERSION: ${MRPT_VERSION}")
#message(STATUS "MRPT_LIBRARIES: ${MRPT_LIBRARIES}")

# Set optimized building:
if(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
endif()

# Declare the target (an executable)
add_executable(mrpt_example1
	test.cpp
	)
target_link_libraries(mrpt_example1 mrpt::slam)
