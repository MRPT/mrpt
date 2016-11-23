INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT application:  GridmapNavSimul
#
#  Run with "cmake ." at the root directory
#
#  May 2008, Jose Luis Blanco <jlblanco@ctima.uma.es>
#-----------------------------------------------------------------
PROJECT(GridmapNavSimul)

IF(CMAKE_MRPT_HAS_WXWIDGETS AND CMAKE_MRPT_HAS_OPENGL_GLUT)

	# ---------------------------------------------
	# TARGET:
	# ---------------------------------------------
	SET(SRCS
		gridmapSimulApp.cpp gridmapSimulApp.h
		gridmapSimulMain.cpp gridmapSimulMain.h
		CAboutBox.cpp CAboutBox.h
		)
	IF(WIN32)
		SET(SRCS ${SRCS} resource.rc)
	ENDIF()

	# Define the executable target:
	ADD_EXECUTABLE(${PROJECT_NAME} WIN32 ${SRCS}  ${MRPT_VERSION_RC_FILE})

	# Add the required libraries for linking:
	INCLUDE_DIRECTORIES( ${wxWidgets_INCLUDE_DIRS} )
	TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${wxWidgets_LIBRARIES} )

	TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${MRPT_LINKER_LIBS})

	# Dependencies on MRPT libraries:
	#  Just mention the top-level dependency, the rest will be detected automatically,
	#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
        DeclareAppDependencies(${PROJECT_NAME} mrpt-maps mrpt-hwdrivers mrpt-kinematics)

	DeclareAppForInstall(${PROJECT_NAME})

	AppStartMenuLink(${PROJECT_NAME} "Gridmap navigation dataset generator")   # Add link to the Start menu (in Windows Installable packages)


ELSE()
	# ----------------------------------------------------------------------------------
    # For convenience. When we cannot continue, inform the user
	# ----------------------------------------------------------------------------------
    MESSAGE(STATUS "wxWidgets and/or GL/GLUT not found!: **This target will not be included in the Makefiles**")
ENDIF()
