#-----------------------------------------------------------------
# CMake file for the MRPT application:  navlog-viewer
#
#  Run with "cmake ." at the root directory
#
#  Feb 2010, Jose Luis Blanco <jlblanco@ctima.uma.es>
#-----------------------------------------------------------------
project(navlog-viewer)

mrpt_return_if_not_wxwidgets()

# ---------------------------------------------
# TARGET:
# ---------------------------------------------
set(SRCS
	navlog_viewer_GUI_designMain.h navlog_viewer_GUI_designMain.cpp
	navlog_viewer_GUI_designApp.h  navlog_viewer_GUI_designApp.cpp
	)

if(WIN32)
	set(SRCS ${SRCS} resource.rc)
endif()

# Define the executable target:
add_executable(${PROJECT_NAME} WIN32 ${SRCS} ${MRPT_VERSION_RC_FILE})

# -ldl required by the plugin load mechanism of this app:
if(UNIX)
	target_link_libraries(${PROJECT_NAME} dl)
endif()

# Add the required libraries for linking:
target_link_libraries(${PROJECT_NAME} ${wxWidgets_LIBRARIES} mrptwxthings)

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${PROJECT_NAME} mrpt::gui mrpt::nav)

DeclareAppForInstall(${PROJECT_NAME})

AppStartMenuLink(${PROJECT_NAME} "Viewer of navigation log files")   # Add link to the Start menu (in Windows Installable packages)

