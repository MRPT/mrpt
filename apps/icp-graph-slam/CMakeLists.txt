INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT application:  icp-graph-slam
#  Feb 2014, Jose Luis Blanco
#-----------------------------------------------------------------
PROJECT(icp_graph_slam)

# ---------------------------------------------
# TARGET:
# ---------------------------------------------
SET(TMP_TARGET_NAME "icp-graph-slam")

# Define the executable target:
ADD_EXECUTABLE(${TMP_TARGET_NAME}
               icp-graph-slam_main.cpp)

# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${MRPT_LINKER_LIBS})

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically, 
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${TMP_TARGET_NAME} mrpt-gui mrpt-graphslam)

DeclareAppForInstall(${TMP_TARGET_NAME})
