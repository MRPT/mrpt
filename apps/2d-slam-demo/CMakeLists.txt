#-----------------------------------------------------------------
# CMake file for the MRPT application:  2d-slam-demo
#
#  Run with "cmake ." at the root directory
#
#  October 2007, Jose Luis Blanco <jlblanco@ctima.uma.es>
#-----------------------------------------------------------------
project(2d-slam-demo)

mrpt_return_if_not_wxwidgets()

# ---------------------------------------------
# TARGET:
# ---------------------------------------------
set(SRCS
	slamdemoApp.cpp   slamdemoApp.h
	slamdemoMain.cpp  slamdemoMain.h
	batch_experiments.cpp
	CDlgParams.cpp CDlgParams.h
	CLogView.cpp CLogView.h
	)

if(WIN32)
	set(SRCS ${SRCS} resource.rc)
endif()

# Define the executable target:
add_executable(${PROJECT_NAME} WIN32 ${SRCS} ${MRPT_VERSION_RC_FILE})

# Add the required libraries for linking:
target_link_libraries(${PROJECT_NAME} ${wxWidgets_LIBRARIES})

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${PROJECT_NAME}
	mrpt::gui
	mrpt::slam
	mrpt::tclap
	)

DeclareAppForInstall(${PROJECT_NAME})

AppStartMenuLink(${PROJECT_NAME} "2D KF-SLAM Simulator")   # Add link to the Start menu (in Windows Installable packages)
