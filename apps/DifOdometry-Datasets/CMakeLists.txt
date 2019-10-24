

#-----------------------------------------------------------------
# CMake file for the MRPT application:  ReactiveNav3D-Demo
#
#  Run with "cmake ." at the root directory
#
#  January 2014, Mariano Jaimez Tarifa <marianojt@uma.es>
#-----------------------------------------------------------------
project(DifOdometry-Datasets)



# ---------------------------------------------
# TARGET:
# ---------------------------------------------
# Define the executable target:
add_executable(${PROJECT_NAME}
			   DifOdometry_Datasets_main.cpp
			   DifOdometry_Datasets.cpp
			   DifOdometry_Datasets.h
			    ${MRPT_VERSION_RC_FILE})

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${PROJECT_NAME}
	mrpt::gui
	mrpt::opengl
	mrpt::vision
	)

DeclareAppForInstall(${PROJECT_NAME})
