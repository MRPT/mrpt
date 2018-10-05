include(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

if (CMAKE_MRPT_HAS_OPENNI2) # This project can't be built without OpenNI2

#-----------------------------------------------------------------
# CMake file for the MRPT application:  ReactiveNav3D-Demo
#
#  Run with "cmake ." at the root directory
#
#  January 2014, Mariano Jaimez Tarifa <marianojt@uma.es>
#-----------------------------------------------------------------
project(DifOdometry-Camera)



include_directories(${OPENNI2_INCLUDE_DIRS})
# Add include directories as "-isystem" to avoid warnings :
ADD_DIRECTORIES_AS_ISYSTEM(OPENNI2_INCLUDE_DIRS)

# ---------------------------------------------
# TARGET:
# ---------------------------------------------
# Define the executable target:
add_executable(${PROJECT_NAME}
			   DifOdometry_Camera_main.cpp
			   DifOdometry_Camera.cpp
			   DifOdometry_Camera.h
			    ${MRPT_VERSION_RC_FILE})

# Add the required libraries for linking:
target_link_libraries(${PROJECT_NAME} ${MRPT_LINKER_LIBS} ${OPENNI2_LIBRARIES})

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${PROJECT_NAME} mrpt-gui mrpt-opengl mrpt-vision)

DeclareAppForInstall(${PROJECT_NAME})

endif (CMAKE_MRPT_HAS_OPENNI2) # This project can't be built without OpenNI2
