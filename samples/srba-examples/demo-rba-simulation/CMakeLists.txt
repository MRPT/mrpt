PROJECT(demo-rba-simulation)

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
FIND_PACKAGE(MRPT REQUIRED srba gui maps vision)

if(MSVC)
	# For MSVC to avoid the C1128 error about too large object files:
	SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /bigobj")
	SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /bigobj")
endif(MSVC)

# Set optimized building in GCC:
IF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -mtune=native")
ENDIF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")


# --------------------------------------------------------------------------
# Declare the app:
# --------------------------------------------------------------------------
FILE(GLOB PROGSRCS "*.cpp")
FILE(GLOB PROGHDRS "*.h")

ADD_EXECUTABLE(demo-rba-simulation ${PROGSRCS} ${PROGHDRS})
TARGET_LINK_LIBRARIES(demo-rba-simulation ${MRPT_LIBS}) # Add the required libraries for linking:


# -------------------------------------------------------------------------
# This part can be removed if you are compiling this program outside of 
#  the MRPT tree:
# -------------------------------------------------------------------------
IF("${CMAKE_PROJECT_NAME}" STREQUAL "MRPT") # Fails if build outside of MRPT project.
	DeclareAppDependencies(demo-rba-simulation  mrpt-srba mrpt-gui mrpt-maps mrpt-vision) # Dependencies
ENDIF("${CMAKE_PROJECT_NAME}" STREQUAL "MRPT")
# -------------------------------------------------------------------------

