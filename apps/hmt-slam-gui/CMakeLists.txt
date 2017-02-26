INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT application:  hmt-slam-gui
#
#  Run with "cmake ." at the root directory
#
#  Apr 2010, Jose Luis Blanco <jlblanco@ctima.uma.es>
#-----------------------------------------------------------------
PROJECT(hmt-slam-gui)

IF(CMAKE_MRPT_HAS_WXWIDGETS AND CMAKE_MRPT_HAS_OPENGL_GLUT)

	# ---------------------------------------------
	# TARGET:
	# ---------------------------------------------
	SET(SRCS
		hmt_slam_guiApp.cpp hmt_slam_guiApp.h
		hmt_slam_guiMain.cpp hmt_slam_guiMain.h
		MyGLCanvas.cpp  MyGLCanvas.h
		MyArtProvider.cpp  MyArtProvider.h
		CDlgLog.cpp CDlgLog.h
		CAboutBox.cpp  CAboutBox.h
		generate_3d_views.cpp
		thread_hmtslam.cpp
		)
	IF(WIN32)
		SET(SRCS ${SRCS} resource.rc)
	ENDIF()

	INCLUDE_DIRECTORIES("${MRPT_SOURCE_DIR}/otherlibs/wxThings/")

	ADD_EXECUTABLE(${PROJECT_NAME} WIN32 ${SRCS} ${MRPT_VERSION_RC_FILE})

	# Add the required libraries for linking:
	INCLUDE_DIRECTORIES( ${wxWidgets_INCLUDE_DIRS} )
	TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${wxWidgets_LIBRARIES} )

	TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${MRPT_LINKER_LIBS})

	#   FabMap module for HMT-SLAM? (optional)
	IF(FabMap_FOUND)
		TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${FabMap_LIBRARIES})
	ENDIF()

	IF(UNIX)
		TARGET_LINK_LIBRARIES(${PROJECT_NAME} mrpt-hmtslam )
	ENDIF()

	TARGET_LINK_LIBRARIES(${PROJECT_NAME} mrptwxthings)

	# Dependencies on MRPT libraries:
	#  Just mention the top-level dependency, the rest will be detected automatically,
	#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
	DeclareAppDependencies(${PROJECT_NAME} mrpt-gui mrpt-hmtslam mrpt-slam mrpt-bayes)

	DeclareAppForInstall(${PROJECT_NAME})

	AppStartMenuLink(${PROJECT_NAME} "HMT-SLAM visual interface")   # Add link to the Start menu (in Windows Installable packages)

ELSE()
	# ----------------------------------------------------------------------------------
	# For convenience. When we cannot continue, inform the user
	# ----------------------------------------------------------------------------------
    MESSAGE(STATUS "wxWidgets and/or GL/GLUT not found!: **This target will not be included in the Makefiles**")
ENDIF()
