INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT application:  RawLogGrabber
#
#  Run with "cmake ." at the root directory
#
#  October 2014, Jesus Briales <jesusbriales@uma.es>
#-----------------------------------------------------------------
PROJECT(MexGrabber)

#To get mex.h to work properly,
#this macro must be defined if and only if the compiled object
#will eventually be linked into a mex file.
ADD_DEFINITIONS(/DMATLAB_MEX_FILE) #define matlab macros
ADD_DEFINITIONS(/DMX_COMPAT_32)

# ---------------------------------------------
# TARGET:
# ---------------------------------------------
# Define the executable target
# (used as test executable):
ADD_EXECUTABLE(mexgrabber-test
               mexgrabber.cpp
              )
SET(TMP_TARGET_NAME "mexgrabber-test")

# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${MRPT_LINKER_LIBS_RELorDEB} )
TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} mrpt-hwdrivers${MRPT_LINKER_LIBS_POSTFIX} )
TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${MATLAB_LIBRARIES} )

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${TMP_TARGET_NAME} mrpt-hwdrivers mrpt-obs)

# ---------------------------------------------
# MEX LIBRARY:
# ---------------------------------------------
# Define the MEX library
# (used by Matlab):
# set up matlab libraries
ADD_LIBRARY(mexgrabber SHARED
            mexgrabber.cpp
            ${CMAKE_CURRENT_LIST_DIR}/Matlabdef.def
            )

set(TMP_MEX_NAME mexgrabber)

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

# TODO: How should these mex libraries be declared?
DeclareAppForInstall(${TMP_MEX_NAME})
DeclareAppDependencies(${TMP_MEX_NAME} mrpt-hwdrivers mrpt-obs)




