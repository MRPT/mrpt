

#-----------------------------------------------------------------
# CMake file for the MRPT application:  graph_slam
#
#  Run with "cmake ." at the root directory
#-----------------------------------------------------------------

if(BUILD_mrpt::graphslam)
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
add_executable(${PROJECT_NAME}
	${SRC_FILES}
	${MRPT_VERSION_RC_FILE}
	)


# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${PROJECT_NAME}
	mrpt::graphslam
	mrpt::graphs
	mrpt::gui
	mrpt::obs
	mrpt::maps
	mrpt::slam
	mrpt::opengl
	mrpt::tclap
	)

DeclareAppForInstall(${PROJECT_NAME})
endif() #BUILD_mrpt::graphslam
