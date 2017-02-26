INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT application:  SceneViewer
#
#  Run with "cmake ." at the root directory
#
#  October 2007, Jose Luis Blanco <jlblanco@ctima.uma.es>
#-----------------------------------------------------------------
PROJECT(SceneViewer3D)

IF(CMAKE_MRPT_HAS_WXWIDGETS AND CMAKE_MRPT_HAS_OPENGL_GLUT)

	# ---------------------------------------------
	# TARGET:
	# ---------------------------------------------
	SET(SRCS
		_DSceneViewerApp.cpp _DSceneViewerApp.h
		_DSceneViewerMain.cpp _DSceneViewerMain.h
		CDialogOptions.cpp   CDialogOptions.h
		CAboutBox.cpp        CAboutBox.h
		CDlgCamTracking.cpp  CDlgCamTracking.h
		CDlgPLYOptions.cpp   CDlgPLYOptions.h
	)
	IF(WIN32)
		SET(SRCS ${SRCS} resource.rc)
	ENDIF()

	INCLUDE_DIRECTORIES("${MRPT_SOURCE_DIR}/otherlibs/wxThings/")

	# Define the executable target:
	ADD_EXECUTABLE(${PROJECT_NAME} WIN32 ${SRCS} ${MRPT_VERSION_RC_FILE})

	# Add the required libraries for linking:
	INCLUDE_DIRECTORIES( ${wxWidgets_INCLUDE_DIRS} )
	TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${wxWidgets_LIBRARIES} )

	TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${MRPT_LINKER_LIBS})

	TARGET_LINK_LIBRARIES(${PROJECT_NAME} mrptwxthings)
	ADD_DEPENDENCIES(${PROJECT_NAME} mrptwxthings)

	# Dependencies on MRPT libraries:
	#  Just mention the top-level dependency, the rest will be detected automatically,
	#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
	DeclareAppDependencies(${PROJECT_NAME} mrpt-opengl mrpt-gui mrpt-maps)

	DeclareAppForInstall(${PROJECT_NAME})

	AppStartMenuLink(${PROJECT_NAME} "Scene Viewer 3D")   # Add link to the Start menu (in Windows Installable packages)

ELSE()
	# ----------------------------------------------------------------------------------
    # For convenience. When we cannot continue, inform the user
	# ----------------------------------------------------------------------------------
    MESSAGE(STATUS "wxWidgets and/or GL/GLUT not found!: **This target will not be included in the Makefiles**")
ENDIF()
