INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT application:  RawLogGrabber
#
#  Run with "cmake ." at the root directory
#
#  October 2007, Jose Luis Blanco <jlblanco@ctima.uma.es>
#-----------------------------------------------------------------
PROJECT(MexGrabber)

#MESSAGE(STATUS "Makefile for application: /apps/mex-grabber ")

# ---------------------------------------------
# MATLAB FLAGS AND CONFIGURATION:
# ---------------------------------------------
set(ENV{MATLAB_ROOT} "/usr/local/MATLAB/R2013a") # To do in a user set input
ADD_DEFINITIONS(/DMATLAB_MEX_FILE) #define matlab macros
ADD_DEFINITIONS(/DMX_COMPAT_32)

FIND_PACKAGE(Matlab REQUIRED) # Matlab
#INCLUDE_DIRECTORIES(${MATLAB_INCLUDE_DIR}) # Done inside
#message("MATLAB_INCLUDE_DIR: ${MATLAB_INCLUDE_DIR}")
INCLUDE_DIRECTORIES("${CMAKE_CURRENT_LIST_DIR}/include/")
#get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
#message("dirs: ${dirs}")

# ---------------------------------------------
# TARGET:
# ---------------------------------------------
# Define the executable target:
ADD_EXECUTABLE(mex-grabber
               mexgrabber_main.cpp
			   )

SET(TMP_TARGET_NAME "mex-grabber")

# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${MRPT_LINKER_LIBS_RELorDEB} )
TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} mrpt-hwdrivers${MRPT_LINKER_LIBS_POSTFIX} )
TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${MATLAB_LIBRARIES} )

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically, 
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${TMP_TARGET_NAME} mrpt-hwdrivers mrpt-obs)

# set up matlab libraries
#message("CMAKE_LIBRARY_OUTPUT_DIRECTORY: ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
#message("LIBRARY_OUTPUT_DIRECTORY: ${LIBRARY_OUTPUT_DIRECTORY}")
ADD_LIBRARY(mexcamera SHARED
		camera_mex.cc
		${CMAKE_CURRENT_LIST_DIR}/Matlabdef.def
		)

set(TMP_MEX_NAME mexcamera)

# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${TMP_MEX_NAME} ${MRPT_LINKER_LIBS_RELorDEB} )
TARGET_LINK_LIBRARIES(${TMP_MEX_NAME} mrpt-hwdrivers${MRPT_LINKER_LIBS_POSTFIX} )
TARGET_LINK_LIBRARIES(${TMP_MEX_NAME} ${MATLAB_LIBRARIES} )

# 32-bit or 64-bit mex
if(WIN32)
  if (CMAKE_CL_64)
	  SET_TARGET_PROPERTIES(${TMP_MEX_NAME} PROPERTIES SUFFIX _.mexw64)
  else(CMAKE_CL_64)
	  SET_TARGET_PROPERTIES(${TMP_MEX_NAME} PROPERTIES SUFFIX _.mexw32)
  endif(CMAKE_CL_64)
else(WIN32)
  if (CMAKE_SIZEOF_VOID_P MATCHES "8")
	  SET_TARGET_PROPERTIES(${TMP_MEX_NAME} PROPERTIES SUFFIX _.mexa64 PREFIX "")
  else(CMAKE_SIZEOF_VOID_P MATCHES "8")
	  SET_TARGET_PROPERTIES(${TMP_MEX_NAME} PROPERTIES SUFFIX _.mexglx PREFIX "")
  endif (CMAKE_SIZEOF_VOID_P MATCHES "8")
endif(WIN32)

DeclareAppForInstall(${TMP_MEX_NAME})
DeclareAppDependencies(${TMP_MEX_NAME} mrpt-hwdrivers mrpt-obs)
