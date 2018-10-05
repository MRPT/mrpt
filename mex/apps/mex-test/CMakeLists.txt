include(../../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT MEX library:  MexTest
#
#  Run with "cmake ." at the root directory
#
#  October 2014, Jesus Briales <jesusbriales@uma.es>
#-----------------------------------------------------------------
project(MexTest)

# ---------------------------------------------
# MEX FUNCTION:
# ---------------------------------------------
# Define the MEX module for Matlab:
add_library(mextest SHARED # MEX must be SHARED
			mex_test.cpp
			${CMAKE_CURRENT_LIST_DIR}/Matlabdef.def
			)

set(TMP_MEX_NAME "mextest")

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

define_mex_lib(${TMP_MEX_NAME})
# No test here because no main() function is declared
