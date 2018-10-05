include(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT application:  hmt-slam-gui
#
#  Run with "cmake ." at the root directory
#
#  Apr 2010, Jose Luis Blanco <jlblanco@ctima.uma.es>
#-----------------------------------------------------------------
project(hmt-slam-gui)

if(CMAKE_MRPT_HAS_WXWIDGETS AND CMAKE_MRPT_HAS_OPENGL_GLUT)

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

	include_directories("${MRPT_SOURCE_DIR}/otherlibs/wxThings/")

	add_executable(${PROJECT_NAME} WIN32 ${SRCS} ${MRPT_VERSION_RC_FILE})

	# Add the required libraries for linking:
	include_directories( ${wxWidgets_INCLUDE_DIRS} )
	target_link_libraries(${PROJECT_NAME} ${wxWidgets_LIBRARIES} )

	target_link_libraries(${PROJECT_NAME} ${MRPT_LINKER_LIBS})

	#   FabMap module for HMT-SLAM? (optional)
	if(FabMap_FOUND)
		target_link_libraries(${PROJECT_NAME} ${FabMap_LIBRARIES})
	endif()

	if(UNIX)
		target_link_libraries(${PROJECT_NAME} mrpt-hmtslam )
	endif()

	target_link_libraries(${PROJECT_NAME} mrptwxthings)

	# Dependencies on MRPT libraries:
	#  Just mention the top-level dependency, the rest will be detected automatically,
	#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
	DeclareAppDependencies(${PROJECT_NAME} mrpt-gui mrpt-hmtslam mrpt-slam mrpt-bayes)

	DeclareAppForInstall(${PROJECT_NAME})

	AppStartMenuLink(${PROJECT_NAME} "HMT-SLAM visual interface")   # Add link to the Start menu (in Windows Installable packages)

else()
	# ----------------------------------------------------------------------------------
	# For convenience. When we cannot continue, inform the user
	# ----------------------------------------------------------------------------------
    message(STATUS "wxWidgets and/or GL/GLUT not found!: **This target will not be included in the Makefiles**")
endif()
