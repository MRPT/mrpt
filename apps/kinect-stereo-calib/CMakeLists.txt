#-----------------------------------------------------------------
# CMake file for the MRPT application:  kinect-calibrate
#
#  Run with "cmake ." at the root directory
#
#  March 2012, Jose Luis Blanco <joseluisblancoc@gmail.com>
#-----------------------------------------------------------------
project(kinect-stereo-calib)

mrpt_return_if_not_wxwidgets()
if (NOT CMAKE_MRPT_HAS_OPENGL_GLUT)
	return()
endif()
# ---------------------------------------------
# TARGET:
# ---------------------------------------------
set(SRCS
	kinect_calibrate_guiApp.cpp kinect_calibrate_guiApp.h
	kinect_calibrate_guiMain.cpp kinect_calibrate_guiMain.h
	MyGLCanvas.cpp MyGLCanvas.h
	)

if(WIN32)
	set(SRCS ${SRCS} resource.rc)
endif()

# Define the executable target:
add_executable(${PROJECT_NAME} WIN32 ${SRCS} ${MRPT_VERSION_RC_FILE})

# Add the required libraries for linking:
target_link_libraries(${PROJECT_NAME} imp_wxwidgets )

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${PROJECT_NAME} mrpt::hwdrivers mrpt::gui mrpt::vision)

DeclareAppForInstall(${PROJECT_NAME})

AppStartMenuLink(${PROJECT_NAME} "Kinect (and generic stereo cams) calibration tool")   # Add link to the Start menu (in Windows Installable packages)

