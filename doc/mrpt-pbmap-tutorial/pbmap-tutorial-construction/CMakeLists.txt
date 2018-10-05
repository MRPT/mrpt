project(pbmap-tutorial-construction)

cmake_minimum_required(VERSION 3.1)
set (CMAKE_CXX_STANDARD 14)  # Require C++14
cmake_policy(SET CMP0003 NEW)  # Required by CMake 2.7+

# The list of "libs" which can be included can be found in:
#  http://www.mrpt.org/Libraries
# Add the top-level dependencies only.
# --------------------------------------------------------------------------
find_package(MRPT REQUIRED pbmap)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

# Define the executable target:
add_executable(${PROJECT_NAME} test.cpp  )

# Add the required libraries for linking:
target_link_libraries(${PROJECT_NAME}
	${MRPT_LIBRARIES}  # This is filled by find_package(MRPT ...)
	${PCL_LIBRARIES}
	)

# Set optimized building:
if((${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang" OR CMAKE_COMPILER_IS_GNUCXX) AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
	add_compile_options(-O3)
endif()
