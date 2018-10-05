# CMake file for the MRPT application:  ptg-configurator
#  Run with "cmake ." at the root directory
include(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

project(ptg-configurator)

if(CMAKE_MRPT_HAS_WXWIDGETS)

	# ---------------------------------------------
	# TARGET:
	# ---------------------------------------------
	set(SRCS
	 ptgConfiguratorApp.cpp   ptgConfiguratorApp.h
	 ptgConfiguratorMain.cpp  ptgConfiguratorMain.h
	 MyGLCanvas.cpp MyGLCanvas.h
	)

	if(WIN32)
		set(SRCS ${SRCS} resource.rc)
	endif()

	# Define the executable target:
	add_executable(${PROJECT_NAME} WIN32 ${SRCS} ${MRPT_VERSION_RC_FILE})

	# Add the required libraries for linking:
	target_link_libraries(${PROJECT_NAME} ${wxWidgets_LIBRARIES} )
	target_link_libraries(${PROJECT_NAME} ${MRPT_LINKER_LIBS})

	# -ldl required by the plugin load mechanism of this app:
	if(UNIX)
	        target_link_libraries(${PROJECT_NAME} dl)
	endif()

	# Dependencies on MRPT libraries:
	#  Just mention the top-level dependency, the rest will be detected automatically,
	#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
	DeclareAppDependencies(${PROJECT_NAME} mrpt-gui mrpt-nav)

	DeclareAppForInstall(${PROJECT_NAME})

	AppStartMenuLink(${PROJECT_NAME} "PTG configurator")   # Add link to the Start menu (in Windows Installable packages)

else()
    message(STATUS "wxWidgets not found!: **This target will not be included in the Makefiles**")
endif()
