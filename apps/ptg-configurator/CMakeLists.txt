# CMake file for the MRPT application:  ptg-configurator
#  Run with "cmake ." at the root directory
INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

PROJECT(ptg-configurator)

IF(CMAKE_MRPT_HAS_WXWIDGETS)

	# ---------------------------------------------
	# TARGET:
	# ---------------------------------------------
	SET(SRCS
	 ptgConfiguratorApp.cpp   ptgConfiguratorApp.h
	 ptgConfiguratorMain.cpp  ptgConfiguratorMain.h
	 CAboutBox.cpp  CAboutBox.h
	 MyGLCanvas.cpp MyGLCanvas.h
	)

	IF(WIN32)
		SET(SRCS ${SRCS} resource.rc)
	ENDIF()

	# Define the executable target:
	ADD_EXECUTABLE(${PROJECT_NAME} WIN32 ${SRCS} ${MRPT_VERSION_RC_FILE})

	# Add the required libraries for linking:
	TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${wxWidgets_LIBRARIES} )
	TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${MRPT_LINKER_LIBS})

	# -ldl required by the plugin load mechanism of this app:
	IF(UNIX)
	        TARGET_LINK_LIBRARIES(${PROJECT_NAME} dl)
	ENDIF()

	# Dependencies on MRPT libraries:
	#  Just mention the top-level dependency, the rest will be detected automatically,
	#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
	DeclareAppDependencies(${PROJECT_NAME} mrpt-gui mrpt-nav)

	DeclareAppForInstall(${PROJECT_NAME})

	AppStartMenuLink(${PROJECT_NAME} "PTG configurator")   # Add link to the Start menu (in Windows Installable packages)

ELSE()
    MESSAGE(STATUS "wxWidgets not found!: **This target will not be included in the Makefiles**")
ENDIF()
