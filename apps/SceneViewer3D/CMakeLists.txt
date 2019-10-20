

#-----------------------------------------------------------------
# CMake file for the MRPT application:  SceneViewer
#
#  Run with "cmake ." at the root directory
#
#  October 2007, Jose Luis Blanco <jlblanco@ctima.uma.es>
#-----------------------------------------------------------------
project(SceneViewer3D)

if(NOT (CMAKE_MRPT_HAS_WXWIDGETS AND CMAKE_MRPT_HAS_OPENGL_GLUT))
	message(STATUS "wxWidgets and/or GL/GLUT not found!: **This target will not be included in the Makefiles**")
	return()
endif()

# ---------------------------------------------
# TARGET:
# ---------------------------------------------
set(SRCS
	_DSceneViewerApp.cpp _DSceneViewerApp.h
	_DSceneViewerMain.cpp _DSceneViewerMain.h
	CDialogOptions.cpp   CDialogOptions.h
	CDlgCamTracking.cpp  CDlgCamTracking.h
	CDlgPLYOptions.cpp   CDlgPLYOptions.h
	)
if(WIN32)
	set(SRCS ${SRCS} resource.rc)
endif()

# Define the executable target:
add_executable(${PROJECT_NAME} WIN32 ${SRCS} ${MRPT_VERSION_RC_FILE})

# Add the required libraries for linking:
target_link_libraries(${PROJECT_NAME}
	imp_wxwidgets
	${MRPT_OPENGL_LIBS}
	${LAS_LIBS}
	mrptwxthings
	)

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${PROJECT_NAME} mrpt::opengl mrpt::gui mrpt::maps)

DeclareAppForInstall(${PROJECT_NAME})

AppStartMenuLink(${PROJECT_NAME} "Scene Viewer 3D")   # Add link to the Start menu (in Windows Installable packages)

