

#-----------------------------------------------------------------
# CMake file for the MRPT application:  hmtMapViewer
#
#  Run with "cmake ." at the root directory
#
#  May 2008, Jose Luis Blanco <jlblanco@ctima.uma.es>
#-----------------------------------------------------------------
project(hmtMapViewer)

if(NOT(CMAKE_MRPT_HAS_WXWIDGETS AND CMAKE_MRPT_HAS_OPENGL_GLUT))
	message(STATUS "wxWidgets and/or GL/GLUT not found!: **This target will not be included in the Makefiles**")
	return()
endif()

# ---------------------------------------------
# TARGET:
# ---------------------------------------------
set(SRCS
	hmtMapViewerApp.cpp hmtMapViewerApp.h
	hmtMapViewerMain.cpp hmtMapViewerMain.h
	)
if(WIN32)
#		set(SRCS ${SRCS} resource.rc)
endif()

# Define the executable target:
add_executable(${PROJECT_NAME} WIN32 ${SRCS} ${MRPT_VERSION_RC_FILE})

# Add the required libraries for linking:
target_link_libraries(${PROJECT_NAME}
	${wxWidgets_LIBRARIES}
	)

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${PROJECT_NAME} mrpt::gui mrpt::slam mrpt::hmtslam)

DeclareAppForInstall(${PROJECT_NAME})

