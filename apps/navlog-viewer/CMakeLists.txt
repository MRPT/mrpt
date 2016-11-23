INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT application:  navlog-viewer
#
#  Run with "cmake ." at the root directory
#
#  Feb 2010, Jose Luis Blanco <jlblanco@ctima.uma.es>
#-----------------------------------------------------------------

PROJECT(navlog-viewer)

IF(CMAKE_MRPT_HAS_WXWIDGETS)
	# ---------------------------------------------
	# TARGET:
	# ---------------------------------------------
	SET(SRCS
		navlog_viewer_GUI_designMain.h navlog_viewer_GUI_designMain.cpp
		navlog_viewer_GUI_designApp.h  navlog_viewer_GUI_designApp.cpp
		CAboutBox.h CAboutBox.cpp
	 	)

	IF(WIN32)
		SET(SRCS ${SRCS} resource.rc)
	ENDIF()

	INCLUDE_DIRECTORIES("${MRPT_SOURCE_DIR}/otherlibs/wxThings/")


	# Define the executable target:
	ADD_EXECUTABLE(${PROJECT_NAME} WIN32 ${SRCS} ${MRPT_VERSION_RC_FILE})

	# -ldl required by the plugin load mechanism of this app:
	IF(UNIX)
	        TARGET_LINK_LIBRARIES(${PROJECT_NAME} dl)
	ENDIF()

	# Add the required libraries for linking:
	TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${wxWidgets_LIBRARIES} )
	TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${MRPT_LINKER_LIBS})

	TARGET_LINK_LIBRARIES(${PROJECT_NAME} mrptwxthings)
	ADD_DEPENDENCIES(${PROJECT_NAME} mrptwxthings)

	# Dependencies on MRPT libraries:
	#  Just mention the top-level dependency, the rest will be detected automatically,
	#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
	DeclareAppDependencies(${PROJECT_NAME} mrpt-gui mrpt-nav)

	DeclareAppForInstall(${PROJECT_NAME})

	AppStartMenuLink(${PROJECT_NAME} "Viewer of navigation log files")   # Add link to the Start menu (in Windows Installable packages)

ELSE()
	# ----------------------------------------------------------------------------------
    	# For convenience. When we cannot continue, inform the user
	# ----------------------------------------------------------------------------------
    MESSAGE(STATUS "wxWidgets not found!: **This target will not be included in the Makefiles**")
ENDIF()
