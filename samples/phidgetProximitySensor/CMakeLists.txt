INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------------------------------------
# CMake file for the MRPT example:  /phidgetProximitySensor
#
#  Run with "ccmake ." at the root directory
#
# FILE AUTOMATICALLY GENERATED FROM THE TREE TOP CMakeLists.txt, 
#  DO NOT MODIFY MANUALLY!!
#
#  January 2008, Jose Luis Blanco <jlblanco@ctima.uma.es>
#-----------------------------------------------------------------------------------------------
#SET(scope )
SET(sampleName phidgetProximitySensor)

# Below is the fixed part of the declaration -------------------
#SET(PRJ_NAME "EXAMPLE_${scope}_${sampleName}")
SET(PRJ_NAME "EXAMPLE_${sampleName}")
PROJECT(${PRJ_NAME})
#MESSAGE(STATUS "Makefile for example: /samples/${sampleName}")

SET(PRJSRCDIR "${PRJ_NAME}_SOURCE_DIR")

# Set the output directory of each sample into its corresponding subdirectory in the binary tree
SET(EXECUTABLE_OUTPUT_PATH "${${${PRJ_NAME}_BINARY_DIR}}")

# ---------------------------------------------
# TARGET:
# ---------------------------------------------
# Define the executable target:
ADD_EXECUTABLE(${sampleName} test.cpp  ) 

#SET_TARGET_PROPERTIES(${sampleName} PROPERTIES PROJECT_LABEL "(EXAMPLE-${scope}) ${sampleName}")
SET_TARGET_PROPERTIES(${sampleName} PROPERTIES PROJECT_LABEL "(EXAMPLE) ${sampleName}")

# Add optional executable flags:
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${MRPT_EXES_CXX_FLAGS}") 

# Add defines, if any:
SET(MY_DEFS )
IF(MY_DEFS) # If not empty
	ADD_DEFINITIONS("-D${MY_DEFS}")
ENDIF(MY_DEFS)


# Add the required libraries for linking:
SET(MY_LIBS "")
LIST(APPEND MY_LIBS "")
IF(MY_LIBS) # If not empty
	TARGET_LINK_LIBRARIES(${sampleName} ${MY_LIBS})
ENDIF(MY_LIBS)

# Dependencies:
DeclareAppDependencies(${sampleName} mrpt-slam;mrpt-hwdrivers)

