#-------------------------------------------------------------------------------
# CMake file for the MRPT example:  /graph_slam_demo
#
#  Run with "ccmake ." at the root directory, or use it as a template for 
#   starting your own programs
#-------------------------------------------------------------------------------

# Sat Apr 30 15:07:52 EEST 2016, nickkouk
#j
# see https://app.asana.com/0/68404150701387/107235506964376

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

#-------------------------------------------------------------------------------
# Declare a new CMake Project:
#-------------------------------------------------------------------------------
set(PROJECT_NAME "graphslam-engine")
project(${PROJECT_NAME})

# version number
set(graphslam-engine_VERSION_MAJOR 0)
set(graphslam-engine_VERSION_MINOR 1)

#if (${UNIX})
  #set (DESKTOP $ENV{HOME})
#else()
  #set (DESKTOP $ENV{USERPROFILE}/Desktop)
#endif()
#message(${DESKTOP})

message("Starting CMake Compilation!")
message("Project: ${PROJECT_NAME}")
message("---------------------------")

## Loops in CMake
#set(a 1)
#set(b "kalimera")
#set(c "kalinuxta")
#foreach(F ${a} ${b} ${c})
    #message(${F})
#endforeach(F)

## macros in cmake
#macro(kalimera name) #macro(macro_name arg1 arg2 ..)
    #message("kalimera ${name}")
    #message("kalinuxta ${name}")
    #message("kalispera ${name}")
    #message("ARGV = ${ARGV}")
    #message("ARGC = ${ARGC}")
    #message("ARGN = ${ARGN}")
#endmacro(kalimera name)
#kalimera("Niko")
#kalimera("Niko" "George" "Spyros") # providing extra arguments


#-------------------------------------------------------------------------------
# Various CMake variables declarations
#-------------------------------------------------------------------------------
set(MRPT_DIR "$ENV{mrpt}")
set(PROJECT_PATH "$ENV{mrpt_project}")
set(INIT_PATH "$ENV{PATH}")

set(SRC_FILES 
    graphslam-engine_app.cpp
    )
set(CMAKE_BINARY_DIR "./build/")
set(REQUIRED_LIBS
    graphslam
    graphs
    gui
    obs
    maps
    hwdrivers 
    slam
    opengl )

# These commands are needed by modern versions of CMake:
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)  # Required by CMake 2.7+
	if(POLICY CMP0043)
        cmake_policy(SET CMP0043 OLD) #  Ignore COMPILE_DEFINITIONS_<Config> properties.
	endif()
endif(COMMAND cmake_policy)

#-------------------------------------------------------------------------------
# set the output directory of each example to its corresponding subdirectory
#  in the binary tree:
#-------------------------------------------------------------------------------
set(EXECUTABLE_OUTPUT_PATH ".")

#-------------------------------------------------------------------------------
#   The list of "libs" which can be included can be found in:
#     http://www.mrpt.org/Libraries
#
#   The dependencies of a library are automatically added, so you only 
#    need to specify the top-most libraries your code depend on.
#-------------------------------------------------------------------------------
FIND_PACKAGE(MRPT REQUIRED ${REQUIRED_LIBS})

# ---------------------------------------------------------------------------
# TARGET:
# ---------------------------------------------------------------------------
# Define the executable target:
ADD_EXECUTABLE(${PROJECT_NAME} ${SRC_FILES}) 

# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${PROJECT_NAME} 
	${MRPT_LIBS}  # This is filled by FIND_PACKAGE(MRPT ...)
	""  # Optional extra libs...
	)

# set optimized building:
IF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
ENDIF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")

# ---------------------------------------------
# This part can be removed if you are compiling this program outside of 
#  the MRPT tree:
# ---------------------------------------------
IF(${CMAKE_PROJECT_NAME} STREQUAL "MRPT") # Fails if build outside of MRPT project.
    DeclareAppDependencies(${PROJECT_NAME} 
        mrpt-graphslam
        mrpt-graphs
        mrpt-gui
        mrpt-obs
        mrpt-hwdrivers #needed for CPlanarLaserScan.h
        mrpt-opengl
        mrpt-slam
        mrpt-maps
        ) # Dependencies
ENDIF(${CMAKE_PROJECT_NAME} STREQUAL "MRPT")
