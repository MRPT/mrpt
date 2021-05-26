project(parse_dataset_example)

cmake_minimum_required(VERSION 3.8)

# --------------------------------------------------------------------------
#   The list of "libs" which can be included can be found in:
#     https://www.mrpt.org/Libraries
#
#   The dependencies of a library are automatically added, so you only
#    need to specify the top-most libraries your code depend on.
# --------------------------------------------------------------------------
find_package( MRPT REQUIRED obs maps topography gui)

# Set optimized building:
if(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")
endif()

# Declare the target (an executable)
add_executable(parse_dataset_example
	main.cpp
	)
target_link_libraries(parse_dataset_example ${MRPT_LIBS})
