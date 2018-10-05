include(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT application:  holonomic-navigator-demo
#
#  Run with "cmake ." at the root directory
#-----------------------------------------------------------------

project(robotic-arm-kinematics)

if(CMAKE_MRPT_HAS_WXWIDGETS)
	# ---------------------------------------------
	# TARGET:
	# ---------------------------------------------
	set(SRCS
	robotic_arm_kinematicsApp.cpp robotic_arm_kinematicsApp.h
	robotic_arm_kinematicsMain.cpp robotic_arm_kinematicsMain.h
	PanelDOF.cpp PanelDOF.h
	CAboutBox.cpp  CAboutBox.h
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

	target_link_libraries(${PROJECT_NAME} mrpt-kinematics)

	target_link_libraries(${PROJECT_NAME} mrptwxthings)
	add_dependencies(${PROJECT_NAME} mrptwxthings)

	# Dependencies on MRPT libraries:
	#  Just mention the top-level dependency, the rest will be detected automatically,
	#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
	DeclareAppDependencies(${PROJECT_NAME} mrpt-kinematics mrpt-gui)

	DeclareAppForInstall(${PROJECT_NAME})


	AppStartMenuLink(${PROJECT_NAME} "GUI for Denavit-Hartenberg parameters robot arm design")   # Add link to the Start menu (in Windows Installable packages)

endif()
