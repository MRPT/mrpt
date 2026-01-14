

if (CMAKE_MRPT_HAS_OPENNI2) # This project can't be built without OpenNI2

#-----------------------------------------------------------------
# CMake file for the MRPT application:  ReactiveNav3D-Demo
#
#  Run with "cmake ." at the root directory
#
#  January 2014, Mariano Jaimez Tarifa <marianojt@uma.es>
#-----------------------------------------------------------------
project(DifOdometry-Camera)


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
if(CMAKE_MRPT_HAS_OPENNI2)
	target_link_libraries(${PROJECT_NAME} ${MRPT_OPENNI2_LIBRARIES})
endif()

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${PROJECT_NAME}
	mrpt::gui
	mrpt::opengl
	mrpt::vision
	)

DeclareAppForInstall(${PROJECT_NAME})

endif (CMAKE_MRPT_HAS_OPENNI2) # This project can't be built without OpenNI2
