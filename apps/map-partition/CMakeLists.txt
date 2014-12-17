INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT application:  map-partition
#
#  Run with "cmake ." at the root directory
#
#  December 2007, Jose Luis Blanco <jlblanco@ctima.uma.es>
#-----------------------------------------------------------------
PROJECT(map_partition)

#MESSAGE(STATUS "Makefile for application: /apps/map-partition")

# ---------------------------------------------
# TARGET:
# ---------------------------------------------
# Define the executable target:
ADD_EXECUTABLE(map-partition
               map-partition.cpp
			    ${MRPT_VERSION_RC_FILE})

SET(TMP_TARGET_NAME "map-partition")



# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${MRPT_LINKER_LIBS})

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically, 
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${TMP_TARGET_NAME} mrpt-gui mrpt-slam mrpt-graphs)


DeclareAppForInstall(${TMP_TARGET_NAME})
