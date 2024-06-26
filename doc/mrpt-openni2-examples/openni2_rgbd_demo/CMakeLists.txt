project(openni2_rgbd_demo)

cmake_minimum_required(VERSION 3.16)

# The list of "libs" which can be included can be found in:
#  https://www.mrpt.org/Libraries
# Add the top-level dependencies only.
# --------------------------------------------------------------------------
find_package(MRPT REQUIRED hwdrivers;gui;opengl;maps)

# Search for libopenni2
include(FindPkgConfig)
pkg_check_modules(PC_OPENNI libopenni2)
include_directories(${PC_OPENNI_INCLUDE_DIRS})
link_directories(${PC_OPENNI_LIBRARY_DIRS})

# Define the executable target:
add_executable(${PROJECT_NAME} test.cpp  )

# Add the required libraries for linking:
target_link_libraries(${PROJECT_NAME}
	${MRPT_LIBRARIES}  # This is filled by find_package(MRPT ...)
	${PC_OPENNI_LIBRARIES}
	)

# Set optimized building:
if((${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang" OR CMAKE_COMPILER_IS_GNUCXX) AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
	add_compile_options(-O3)
endif()
