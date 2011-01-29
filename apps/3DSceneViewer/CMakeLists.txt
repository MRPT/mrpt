INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT application:  SceneViewer
#
#  Run with "cmake ." at the root directory
#
#  October 2007, Jose Luis Blanco <jlblanco@ctima.uma.es>
#-----------------------------------------------------------------
PROJECT(SceneViewer3D)

IF(wxWidgets_FOUND AND CMAKE_MRPT_HAS_OPENGL_GLUT)

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
		../wx-common/CMyRedirector.h
	)
	IF(WIN32)
		SET(SRCS ${SRCS} resource.rc)
	ENDIF(WIN32)

	# Define the executable target:
	ADD_EXECUTABLE(SceneViewer3D WIN32 ${SRCS})

	SET(TMP_TARGET_NAME "SceneViewer3D")


	SET_TARGET_PROPERTIES(${TMP_TARGET_NAME} PROPERTIES PROJECT_LABEL "(APP) ${TMP_TARGET_NAME}")

	# Add the required libraries for linking:
	INCLUDE_DIRECTORIES( ${wxWidgets_INCLUDE_DIRS} )
	TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${wxWidgets_LIBRARIES} )

	TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${MRPT_LINKER_LIBS})

	# Dependencies on MRPT libraries:
	#  Just mention the top-level dependency, the rest will be detected automatically, 
	#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
	DeclareAppDependencies(${TMP_TARGET_NAME} mrpt-opengl mrpt-gui)

	# ----------------------------------------------------------------------------
	#   				INSTALL
	# ----------------------------------------------------------------------------
	INSTALL(TARGETS ${TMP_TARGET_NAME}
		RUNTIME DESTINATION ${mrpt_apps_INSTALL_PREFIX}bin
		LIBRARY DESTINATION ${mrpt_apps_INSTALL_PREFIX}lib${LIB_SUFFIX}
		ARCHIVE DESTINATION ${mrpt_apps_INSTALL_PREFIX}lib${LIB_SUFFIX})


ELSE(wxWidgets_FOUND AND CMAKE_MRPT_HAS_OPENGL_GLUT)
	# ----------------------------------------------------------------------------------
    # For convenience. When we cannot continue, inform the user
	# ----------------------------------------------------------------------------------
    MESSAGE(STATUS "wxWidgets and/or GL/GLUT not found!: **This target will not be included in the Makefiles**")
ENDIF(wxWidgets_FOUND AND CMAKE_MRPT_HAS_OPENGL_GLUT)


