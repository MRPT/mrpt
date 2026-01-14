#-----------------------------------------------------------------
# CMake file for the MRPT application:  holonomic-navigator-demo
#
#  Run with "cmake ." at the root directory
#-----------------------------------------------------------------
project(robotic-arm-kinematics)

mrpt_return_if_not_wxwidgets()
if (NOT CMAKE_MRPT_HAS_OPENGL_GLUT)
	return()
endif()

# ---------------------------------------------
# TARGET:
# ---------------------------------------------
set(SRCS
	robotic_arm_kinematicsApp.cpp robotic_arm_kinematicsApp.h
	robotic_arm_kinematicsMain.cpp robotic_arm_kinematicsMain.h
	PanelDOF.cpp PanelDOF.h
	CAboutBox.cpp  CAboutBox.h
	MyGLCanvas.cpp MyGLCanvas.h
	)

if(WIN32)
	set(SRCS ${SRCS} resource.rc)
endif()

# Define the executable target:
add_executable(${PROJECT_NAME} WIN32 ${SRCS} ${MRPT_VERSION_RC_FILE})

# Add the required libraries for linking:
target_link_libraries(${PROJECT_NAME}
	imp_wxwidgets
	mrptwxthings
	)

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${PROJECT_NAME} mrpt::kinematics mrpt::gui)

DeclareAppForInstall(${PROJECT_NAME})


AppStartMenuLink(${PROJECT_NAME} "GUI for Denavit-Hartenberg parameters robot arm design")   # Add link to the Start menu (in Windows Installable packages)
