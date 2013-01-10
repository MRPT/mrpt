PROJECT(srba-tutorials)

CMAKE_MINIMUM_REQUIRED(VERSION 2.4)
if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)  # Required by CMake 2.7+
endif(COMMAND cmake_policy)

# --------------------------------------------------------------------------
#   The list of "libs" which can be included can be found in:
#     http://www.mrpt.org/Libraries
#   The dependencies of a library are automatically added, so you only 
#    need to specify the top-most libraries your code depend on.
# --------------------------------------------------------------------------
FIND_PACKAGE(MRPT REQUIRED srba gui)

if(MSVC)
	# For MSVC to avoid the C1128 error about too large object files:
	SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /bigobj")
	SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /bigobj")
endif(MSVC)

# Set optimized building in GCC:
IF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -mtune=native")
ENDIF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")


# --------------------------------------------------------------------
#  tutorial-srba-monocular-se3
# --------------------------------------------------------------------
ADD_EXECUTABLE(tutorial-srba-monocular-se3 tutorial-srba-monocular-se3.cpp)
TARGET_LINK_LIBRARIES(tutorial-srba-monocular-se3 ${MRPT_LIBS}) # Add the required libraries for linking:

# --------------------------------------------------------------------
#  tutorial-srba-stereo-se3
# --------------------------------------------------------------------
ADD_EXECUTABLE(tutorial-srba-stereo-se3 tutorial-srba-stereo-se3.cpp)
TARGET_LINK_LIBRARIES(tutorial-srba-stereo-se3 ${MRPT_LIBS})  # Add the required libraries for linking:
