# CMake file for the MRPT application:  ptg-configurator
#  Run with "cmake ." at the root directory

project(ptg-configurator)

mrpt_return_if_not_wxwidgets()
if (NOT CMAKE_MRPT_HAS_OPENGL_GLUT)
	return()
endif()
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
target_link_libraries(${PROJECT_NAME} imp_wxwidgets )

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${PROJECT_NAME} mrpt::gui mrpt::nav)

DeclareAppForInstall(${PROJECT_NAME})

AppStartMenuLink(${PROJECT_NAME} "PTG configurator")   # Add link to the Start menu (in Windows Installable packages)
