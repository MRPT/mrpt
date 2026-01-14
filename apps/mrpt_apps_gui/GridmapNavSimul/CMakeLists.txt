

#-----------------------------------------------------------------
# CMake file for the MRPT application:  GridmapNavSimul
#
#  Run with "cmake ." at the root directory
#
#  May 2008, Jose Luis Blanco <jlblanco@ctima.uma.es>
#-----------------------------------------------------------------
project(GridmapNavSimul)

if(NOT (CMAKE_MRPT_HAS_WXWIDGETS AND CMAKE_MRPT_HAS_OPENGL_GLUT))
	message(STATUS "wxWidgets and/or GL/GLUT not found!: **This target will not be included in the Makefiles**")
	return()
endif()

# ---------------------------------------------
# TARGET:
# ---------------------------------------------
set(SRCS
	gridmapSimulApp.cpp gridmapSimulApp.h
	gridmapSimulMain.cpp gridmapSimulMain.h
	)
if(WIN32)
	set(SRCS ${SRCS} resource.rc)
endif()

# Define the executable target:
add_executable(${PROJECT_NAME} WIN32 ${SRCS}  ${MRPT_VERSION_RC_FILE})

# Add the required libraries for linking:
target_link_libraries(${PROJECT_NAME} imp_wxwidgets )

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${PROJECT_NAME} mrpt::maps mrpt::hwdrivers mrpt::kinematics)

DeclareAppForInstall(${PROJECT_NAME})

AppStartMenuLink(${PROJECT_NAME} "Gridmap navigation dataset generator")   # Add link to the Start menu (in Windows Installable packages)
