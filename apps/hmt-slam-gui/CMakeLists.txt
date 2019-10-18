

#-----------------------------------------------------------------
# CMake file for the MRPT application:  hmt-slam-gui
#
#  Run with "cmake ." at the root directory
#
#  Apr 2010, Jose Luis Blanco <jlblanco@ctima.uma.es>
#-----------------------------------------------------------------
project(hmt-slam-gui)

if(NOT (CMAKE_MRPT_HAS_WXWIDGETS AND CMAKE_MRPT_HAS_OPENGL_GLUT))
	message(STATUS "wxWidgets and/or GL/GLUT not found!: **This target will not be included in the Makefiles**")
	return()
endif()

# ---------------------------------------------
# TARGET:
# ---------------------------------------------
set(SRCS
	hmt_slam_guiApp.cpp hmt_slam_guiApp.h
	hmt_slam_guiMain.cpp hmt_slam_guiMain.h
	MyGLCanvas.cpp  MyGLCanvas.h
	MyArtProvider.cpp  MyArtProvider.h
	CDlgLog.cpp CDlgLog.h
	generate_3d_views.cpp
	thread_hmtslam.cpp
	)
if(WIN32)
	set(SRCS ${SRCS} resource.rc)
endif()

add_executable(${PROJECT_NAME} WIN32 ${SRCS} ${MRPT_VERSION_RC_FILE})

# Add the required libraries for linking:
target_link_libraries(${PROJECT_NAME}
	${wxWidgets_LIBRARIES}
	mrptwxthings
	)

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${PROJECT_NAME} mrpt::gui mrpt::hmtslam mrpt::slam mrpt::bayes)

DeclareAppForInstall(${PROJECT_NAME})

AppStartMenuLink(${PROJECT_NAME} "HMT-SLAM visual interface")   # Add link to the Start menu (in Windows Installable packages)
