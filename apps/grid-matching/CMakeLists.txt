INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT application:  grid-matching
#
#  Run with "cmake ." at the root directory
#
#  October 2007, Jose Luis Blanco <jlblanco@ctima.uma.es>
#-----------------------------------------------------------------
PROJECT(grid_matching)

#MESSAGE(STATUS "Makefile for application: /apps/grid-matching ")

# ---------------------------------------------
# TARGET:
# ---------------------------------------------
# Define the executable target:
ADD_EXECUTABLE(grid-matching
			   grid-matching_main.cpp
			    ${MRPT_VERSION_RC_FILE})

SET(TMP_TARGET_NAME     "grid-matching")
SET(TMP_TARGET_NAME_OUT "grid-matching")

SET_TARGET_PROPERTIES(${TMP_TARGET_NAME} PROPERTIES OUTPUT_NAME ${TMP_TARGET_NAME_OUT})

# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${MRPT_LINKER_LIBS})

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically, 
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${TMP_TARGET_NAME} mrpt-slam mrpt-gui)

DeclareAppForInstall(${TMP_TARGET_NAME})
