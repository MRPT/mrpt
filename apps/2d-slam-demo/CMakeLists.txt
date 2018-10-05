include(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT application:  2d-slam-demo
#
#  Run with "cmake ." at the root directory
#
#  October 2007, Jose Luis Blanco <jlblanco@ctima.uma.es>
#-----------------------------------------------------------------
project(2d-slam-demo)

if(CMAKE_MRPT_HAS_WXWIDGETS)

	# ---------------------------------------------
	# TARGET:
	# ---------------------------------------------
	set(SRCS
		slamdemoApp.cpp   slamdemoApp.h
		slamdemoMain.cpp  slamdemoMain.h
		batch_experiments.cpp
		CDlgParams.cpp CDlgParams.h
		CLogView.cpp CLogView.h
	 )

	if(WIN32)
		set(SRCS ${SRCS} resource.rc)
	endif()

	# Define the executable target:
	add_executable(${PROJECT_NAME} WIN32 ${SRCS} ${MRPT_VERSION_RC_FILE})


	# Add the required libraries for linking:
	include_directories( ${wxWidgets_INCLUDE_DIRS} )
	target_link_libraries(${PROJECT_NAME} ${wxWidgets_LIBRARIES} )
	target_link_libraries(${PROJECT_NAME} ${MRPT_LINKER_LIBS})

	# Dependencies on MRPT libraries:
	#  Just mention the top-level dependency, the rest will be detected automatically,
	#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
	DeclareAppDependencies(${PROJECT_NAME} mrpt-slam mrpt-gui mrpt-tclap)

	DeclareAppForInstall(${PROJECT_NAME})

	AppStartMenuLink(${PROJECT_NAME} "2D KF-SLAM Simulator")   # Add link to the Start menu (in Windows Installable packages)

else()
	# ----------------------------------------------------------------------------------
    # For convenience. When we cannot continue, inform the user
	# ----------------------------------------------------------------------------------
    message(STATUS "wxWidgets not found!: **This target will not be included in the Makefiles**")
endif()
