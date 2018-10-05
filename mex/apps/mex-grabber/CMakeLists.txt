include(../../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT MEX library:  MexGrabber
#
#  Run with "cmake ." at the root directory
#
#  October 2014, Jesus Briales <jesusbriales@uma.es>
#-----------------------------------------------------------------
project(MexGrabber)

# ---------------------------------------------
# MEX CLASS LIBRARY:
# ---------------------------------------------
# Define the MEX module for Matlab:
add_library(mexgrabber SHARED # MEX must be SHARED
			mexgrabber.cpp
			${CMAKE_CURRENT_LIST_DIR}/Matlabdef.def
			)

set(TMP_MEX_NAME "mexgrabber")

# Add the required libraries for linking:
target_link_libraries(${TMP_MEX_NAME} 
	${MRPT_LINKER_LIBS_RELorDEB}
	mrpt-hwdrivers${MRPT_LINKER_LIBS_POSTFIX}
	${MATLAB_LIBRARIES} 
	)

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${TMP_MEX_NAME} mrpt-hwdrivers mrpt-obs)
DeclareAppForInstall(${TMP_MEX_NAME})

# Apply macros to set MEX library custom properties
define_mex_lib_private(${TMP_MEX_NAME})
define_mex_test(${TMP_MEX_NAME}) # Apply after MEX library is COMPLETELY defined to add debuggable executable test
