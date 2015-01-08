INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT application:  ReactiveNav3D-Demo
#
#  Run with "cmake ." at the root directory
#
#  January 2014, Mariano Jaimez Tarifa <marianojt@uma.es>
#-----------------------------------------------------------------
PROJECT(DifOdometry-Datasets)

#MESSAGE(STATUS "Makefile for application: /apps/ReactiveNav3D-Demo ")

# ---------------------------------------------
# TARGET:
# ---------------------------------------------
# Define the executable target:
ADD_EXECUTABLE(DifOdometry-Datasets
               DifOdometry_Datasets_main.cpp
			   DifOdometry_Datasets.cpp
			   DifOdometry_Datasets.h
			    ${MRPT_VERSION_RC_FILE})

SET(TMP_TARGET_NAME "DifOdometry-Datasets")



# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${MRPT_LINKER_LIBS})

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically, 
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${TMP_TARGET_NAME} mrpt-gui mrpt-opengl mrpt-vision)

DeclareAppForInstall(${TMP_TARGET_NAME})


