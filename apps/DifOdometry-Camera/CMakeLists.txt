INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

if (CMAKE_MRPT_HAS_OPENNI2) # This project can't be built without OpenNI2

#-----------------------------------------------------------------
# CMake file for the MRPT application:  ReactiveNav3D-Demo
#
#  Run with "cmake ." at the root directory
#
#  January 2014, Mariano Jaimez Tarifa <marianojt@uma.es>
#-----------------------------------------------------------------
PROJECT(DifOdometry-Camera)

#MESSAGE(STATUS "Makefile for application: /apps/ReactiveNav3D-Demo ")

include_directories(${OPENNI2_INCLUDE_DIRS})
# Add include directories as "-isystem" to avoid warnings :
ADD_DIRECTORIES_AS_ISYSTEM(OPENNI2_INCLUDE_DIRS)

# ---------------------------------------------
# TARGET:
# ---------------------------------------------
# Define the executable target:
ADD_EXECUTABLE(DifOdometry-Camera
               DifOdometry_Camera_main.cpp
			   DifOdometry_Camera.cpp
			   DifOdometry_Camera.h
			    ${MRPT_VERSION_RC_FILE})

SET(TMP_TARGET_NAME "DifOdometry-Camera")



# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${MRPT_LINKER_LIBS} ${OPENNI2_LIBRARIES})

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically, 
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${TMP_TARGET_NAME} mrpt-gui mrpt-opengl mrpt-vision)

DeclareAppForInstall(${TMP_TARGET_NAME})

endif (CMAKE_MRPT_HAS_OPENNI2) # This project can't be built without OpenNI2

