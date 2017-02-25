INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT application:  holonomic-navigator-demo
#
#  Run with "cmake ." at the root directory
#-----------------------------------------------------------------

PROJECT(holonomic-navigator-demo)

IF(CMAKE_MRPT_HAS_WXWIDGETS)

	# ---------------------------------------------
	# TARGET:
	# ---------------------------------------------
	SET(SRCS
	holonomic_navigator_demoApp.cpp holonomic_navigator_demoApp.h
	holonomic_navigator_demoMain.cpp holonomic_navigator_demoMain.h
	../ReactiveNavigationDemo/DEFAULT_GRIDMAP_DATA.h
	CAboutBox.cpp  CAboutBox.h
	MyGLCanvas.cpp MyGLCanvas.h
	)

	IF(WIN32)
		SET(SRCS ${SRCS} resource.rc)
	ENDIF()

	INCLUDE_DIRECTORIES("${MRPT_SOURCE_DIR}/otherlibs/wxThings/")

	# Define the executable target:
	ADD_EXECUTABLE(${PROJECT_NAME} WIN32 ${SRCS} ${MRPT_VERSION_RC_FILE})

	# Add the required libraries for linking:
	TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${wxWidgets_LIBRARIES} )
	TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${MRPT_LINKER_LIBS})

	TARGET_LINK_LIBRARIES(${PROJECT_NAME} mrptwxthings)
	ADD_DEPENDENCIES(${PROJECT_NAME} mrptwxthings)

	# Dependencies on MRPT libraries:
	#  Just mention the top-level dependency, the rest will be detected automatically,
	#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
	DeclareAppDependencies(${PROJECT_NAME} mrpt-nav mrpt-gui)

	DeclareAppForInstall(${PROJECT_NAME})

	AppStartMenuLink(${PROJECT_NAME} "Holonomic navigation simulator")   # Add link to the Start menu (in Windows Installable packages)

ENDIF()
