INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT application:  graph_slam
#
#  Run with "cmake ." at the root directory
#
#  Aug 2010, Jose Luis Blanco <jlblanco@ctima.uma.es>
#-----------------------------------------------------------------
PROJECT(graph_slam)

# ---------------------------------------------
# TARGET:
# ---------------------------------------------
# Define the executable target:
ADD_EXECUTABLE(graph-slam
	graph-slam_main.cpp
	display_graph.cpp
	 ${MRPT_VERSION_RC_FILE}
	)

SET(TMP_TARGET_NAME "graph-slam")



# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${MRPT_LINKER_LIBS})

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${TMP_TARGET_NAME} mrpt-graphslam mrpt-gui)

DeclareAppForInstall(${TMP_TARGET_NAME})
