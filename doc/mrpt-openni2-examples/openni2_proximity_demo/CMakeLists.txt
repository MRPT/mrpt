PROJECT(openni2_proximity_demo)

CMAKE_MINIMUM_REQUIRED(VERSION 3.1)
set (CMAKE_CXX_STANDARD 14)  # Require C++14
cmake_policy(SET CMP0003 NEW)  # Required by CMake 2.7+

# The list of "libs" which can be included can be found in:
#  http://www.mrpt.org/Libraries
# Add the top-level dependencies only.
# --------------------------------------------------------------------------
FIND_PACKAGE(MRPT REQUIRED hwdrivers;gui;opengl;maps)

# Search for libopenni2
include(FindPkgConfig)
pkg_check_modules(PC_OPENNI libopenni2)
include_directories(${PC_OPENNI_INCLUDE_DIRS})
link_directories(${PC_OPENNI_LIBRARY_DIRS})

# Define the executable target:
ADD_EXECUTABLE(${PROJECT_NAME} test.cpp  )

# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${PROJECT_NAME}
	${MRPT_LIBRARIES}  # This is filled by FIND_PACKAGE(MRPT ...)
	${PC_OPENNI_LIBRARIES}
	)

# Set optimized building:
IF((${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang" OR CMAKE_COMPILER_IS_GNUCXX) AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
	add_compile_options(-O3)
ENDIF()
