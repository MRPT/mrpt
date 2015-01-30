#-----------------------------------------------------------------------------------------------
# CMake file for the MRPT example:  /bayesianTracking
#
#  Run with "ccmake ." at the root directory, or use it as a template for 
#   starting your own programs
#-----------------------------------------------------------------------------------------------
SET(sampleName bayesianTracking)
SET(PRJ_NAME "EXAMPLE_${sampleName}")

# ---------------------------------------
# Declare a new CMake Project:
# ---------------------------------------
PROJECT(${PRJ_NAME})

# These commands are needed by modern versions of CMake:
CMAKE_MINIMUM_REQUIRED(VERSION 2.4)
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)  # Required by CMake 2.7+
	if(POLICY CMP0043)
		cmake_policy(SET CMP0043 OLD) #  Ignore COMPILE_DEFINITIONS_<Config> properties.
	endif()
endif(COMMAND cmake_policy)

# ---------------------------------------------------------------------------
# Set the output directory of each example to its corresponding subdirectory
#  in the binary tree:
# ---------------------------------------------------------------------------
SET(EXECUTABLE_OUTPUT_PATH ".")

# --------------------------------------------------------------------------
#   The list of "libs" which can be included can be found in:
#     http://www.mrpt.org/Libraries
#
#   The dependencies of a library are automatically added, so you only 
#    need to specify the top-most libraries your code depend on.
# --------------------------------------------------------------------------
FIND_PACKAGE(MRPT REQUIRED base;bayes;obs;gui)

# ---------------------------------------------
# TARGET:
# ---------------------------------------------
# Define the executable target:
ADD_EXECUTABLE(${sampleName} test.cpp  ) 

SET_TARGET_PROPERTIES(
	${sampleName} 
	PROPERTIES 
	PROJECT_LABEL "(EXAMPLE) ${sampleName}")

# Add special defines needed by this example, if any:
SET(MY_DEFS )
IF(MY_DEFS) # If not empty
	ADD_DEFINITIONS("-D${MY_DEFS}")
ENDIF(MY_DEFS)

# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${sampleName} 
	${MRPT_LIBS}  # This is filled by FIND_PACKAGE(MRPT ...)
	""  # Optional extra libs...
	)

# Set optimized building:
IF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
ENDIF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")


# -------------------------------------------------------------------------
# This part can be removed if you are compiling this program outside of 
#  the MRPT tree:
# -------------------------------------------------------------------------
IF(${CMAKE_PROJECT_NAME} STREQUAL "MRPT") # Fails if build outside of MRPT project.
	DeclareAppDependencies(${sampleName} mrpt-base;mrpt-bayes;mrpt-obs;mrpt-gui) # Dependencies
ENDIF(${CMAKE_PROJECT_NAME} STREQUAL "MRPT")
# -------------------------------------------------------------------------

