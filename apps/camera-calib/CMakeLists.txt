#-----------------------------------------------------------------
# CMake file for the MRPT application:  RawLogViewer
#
#  Run with "cmake ." at the root directory
#
#  October 2007, Jose Luis Blanco <jlblanco@ctima.uma.es>
#-----------------------------------------------------------------
project(camera-calib)

mrpt_return_if_not_wxwidgets()
if (NOT CMAKE_MRPT_HAS_OPENGL_GLUT)
	return()
endif()

# ---------------------------------------------
# TARGET:
# ---------------------------------------------
set(SRCS
	camera_calib_guiApp.cpp   camera_calib_guiApp.h
	camera_calib_guiMain.cpp  camera_calib_guiMain.h
	CDlgCalibWizardOnline.cpp  CDlgCalibWizardOnline.h
	CDlgPoseEst.cpp CDlgPoseEst.h
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
DeclareAppDependencies(${PROJECT_NAME} mrpt::vision mrpt::gui mrpt::hwdrivers)

DeclareAppForInstall(${PROJECT_NAME})

AppStartMenuLink(${PROJECT_NAME} "Camera calibration GUI")   # Add link to the Start menu (in Windows Installable packages)
