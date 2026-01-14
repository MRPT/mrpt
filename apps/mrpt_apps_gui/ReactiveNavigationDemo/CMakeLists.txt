#-----------------------------------------------------------------
# CMake file for the MRPT application:  ReactiveNavigationDemo
#
#  Run with "cmake ." at the root directory
#
#  October 2007, Jose Luis Blanco <jlblanco@ctima.uma.es>
#-----------------------------------------------------------------

project(ReactiveNavigationDemo)

mrpt_return_if_not_wxwidgets()
if (NOT CMAKE_MRPT_HAS_OPENGL_GLUT)
	return()
endif()

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

# Define the executable target:
add_executable(${PROJECT_NAME} WIN32 ${SRCS} ${MRPT_VERSION_RC_FILE})

# Add the required libraries for linking:
target_link_libraries(${PROJECT_NAME} imp_wxwidgets mrptwxthings)

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${PROJECT_NAME} mrpt::gui mrpt::nav mrpt::kinematics)

DeclareAppForInstall(${PROJECT_NAME})

AppStartMenuLink(${PROJECT_NAME} "Reactive navigation demo")   # Add link to the Start menu (in Windows Installable packages)

