INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT application:  graph_slam
#
#  Run with "cmake ." at the root directory
#-----------------------------------------------------------------

IF(BUILD_mrpt-graphslam)
#-------------------------------------------------------------------------------
# Declare a new CMake Project:
#-------------------------------------------------------------------------------
set(PROJECT_NAME "graphslam-engine")
project(${PROJECT_NAME})

# ---------------------------------------------
# TARGET:
# ---------------------------------------------

set(SRC_FILES
    graphslam-engine_app.cpp
    )

# Define the executable target:
ADD_EXECUTABLE(${PROJECT_NAME}
	${SRC_FILES}
	${MRPT_VERSION_RC_FILE}
	)


# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${MRPT_LINKER_LIBS})


# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${PROJECT_NAME}
	mrpt-graphslam
	mrpt-graphs
	mrpt-gui
	mrpt-obs
	mrpt-maps
	mrpt-slam
	mrpt-opengl
	)

DeclareAppForInstall(${PROJECT_NAME})
ENDIF() #BUILD_mrpt-graphslam
