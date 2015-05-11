INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT application:  ReactiveNav3D-Demo
#
#  Run with "cmake ." at the root directory
#
#  Juin 2013, Mariano Jaimez Tarifa <marianojt@uma.es>
#-----------------------------------------------------------------
PROJECT(ReactiveNav3D-Demo)

#MESSAGE(STATUS "Makefile for application: /apps/ReactiveNav3D-Demo ")

# ---------------------------------------------
# TARGET:
# ---------------------------------------------
# Define the executable target:
ADD_EXECUTABLE(ReactiveNav3D-Demo
               ReactiveNav3D_demo_main.cpp
			   ReactiveNav3D_demo.h
			    ${MRPT_VERSION_RC_FILE})

SET(TMP_TARGET_NAME "ReactiveNav3D-Demo")



# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${MRPT_LINKER_LIBS})

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically, 
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${TMP_TARGET_NAME} mrpt-gui mrpt-slam mrpt-nav)

DeclareAppForInstall(${TMP_TARGET_NAME})



#FIND_PACKAGE( MRPT REQUIRED base gui opengl slam maps reactivenav)


