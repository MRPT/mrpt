INCLUDE(../../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT application:  RawLogGrabber
#
#  Run with "cmake ." at the root directory
#
#  October 2007, Jose Luis Blanco <jlblanco@ctima.uma.es>
#-----------------------------------------------------------------
PROJECT(MexTest)

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

# MEX library for testing experimental simple programs
#------------------------------------------------------
ADD_LIBRARY(mextest SHARED
                mex_test.cpp
                ${CMAKE_CURRENT_LIST_DIR}/Matlabdef.def
                )

set(TMP_MEX_NAME mextest)

# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${TMP_MEX_NAME} ${MRPT_LINKER_LIBS_RELorDEB} )
TARGET_LINK_LIBRARIES(${TMP_MEX_NAME} mrpt-hwdrivers${MRPT_LINKER_LIBS_POSTFIX} )
TARGET_LINK_LIBRARIES(${TMP_MEX_NAME} ${MATLAB_LIBRARIES} )

define_mex_lib(${TMP_MEX_NAME})

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${TMP_MEX_NAME} mrpt-hwdrivers mrpt-obs)
