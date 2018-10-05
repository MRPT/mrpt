include(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT application:  ReactiveNavigationDemo
#
#  Run with "cmake ." at the root directory
#
#  October 2007, Jose Luis Blanco <jlblanco@ctima.uma.es>
#-----------------------------------------------------------------

project(ReactiveNavigationDemo)

if(CMAKE_MRPT_HAS_WXWIDGETS)

	# ---------------------------------------------
	# TARGET:
	# ---------------------------------------------
	set(SRCS
	 reactive_navigator_demoApp.cpp  reactive_navigator_demoApp.h
	 reactive_navigator_demoMain.cpp  reactive_navigator_demoMain.h
	 DEFAULT_GRIDMAP_DATA.h
	 MyGLCanvas.cpp MyGLCanvas.h
	)

	if(WIN32)
		set(SRCS ${SRCS} resource.rc)
	endif()

	include_directories("${MRPT_SOURCE_DIR}/otherlibs/wxThings/")

	# Define the executable target:
	add_executable(${PROJECT_NAME} WIN32 ${SRCS} ${MRPT_VERSION_RC_FILE})

	# Add the required libraries for linking:
	target_link_libraries(${PROJECT_NAME} ${wxWidgets_LIBRARIES} )
	target_link_libraries(${PROJECT_NAME} ${MRPT_LINKER_LIBS})
	target_link_libraries(${PROJECT_NAME} mrptwxthings)
	add_dependencies(${PROJECT_NAME} mrptwxthings)

	# Dependencies on MRPT libraries:
	#  Just mention the top-level dependency, the rest will be detected automatically,
	#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
	DeclareAppDependencies(${PROJECT_NAME} mrpt-gui mrpt-nav mrpt-kinematics)

	DeclareAppForInstall(${PROJECT_NAME})

	AppStartMenuLink(${PROJECT_NAME} "Reactive navigation demo")   # Add link to the Start menu (in Windows Installable packages)

else()
	# ----------------------------------------------------------------------------------
    # For convenience. When we cannot continue, inform the user
	# ----------------------------------------------------------------------------------
    message(STATUS "wxWidgets not found!: **This target will not be included in the Makefiles**")
endif()
