#-----------------------------------------------------------------
# CMake file for the MRPT application:  navlog-viewer
#
#  Run with "cmake ." at the root directory
#-----------------------------------------------------------------
project(navlog-viewer)

mrpt_return_if_not_nanogui()

# Define the executable target:
add_executable(${PROJECT_NAME}
	main.cpp
	navlog-viewer-ui.cpp
	navlog-viewer-ui.h
	${MRPT_VERSION_RC_FILE}
)

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${PROJECT_NAME} mrpt::gui mrpt::nav mrpt::tclap)
DeclareAppForInstall(${PROJECT_NAME})
AppStartMenuLink(${PROJECT_NAME} "Viewer of navigation log files")   # Add link to the Start menu (in Windows Installable packages)
