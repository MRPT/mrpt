PROJECT(srba-tutorials)

CMAKE_MINIMUM_REQUIRED(VERSION 2.4)
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)  # Required by CMake 2.7+
	if(POLICY CMP0043)
		cmake_policy(SET CMP0043 OLD) #  Ignore COMPILE_DEFINITIONS_<Config> properties.
	endif()
endif(COMMAND cmake_policy)

# --------------------------------------------------------------------------
#   The list of "libs" which can be included can be found in:
#     http://www.mrpt.org/Libraries
#   The dependencies of a library are automatically added, so you only 
#    need to specify the top-most libraries your code depend on.
# --------------------------------------------------------------------------
FIND_PACKAGE(MRPT REQUIRED srba gui)

if(MSVC)
	# For MSVC to avoid the C1128 error about too large object files:
	SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /bigobj")
	SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /bigobj")
endif(MSVC)

# Set optimized building in GCC:
IF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
ENDIF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")


MACRO(DEFINE_TUTORIAL_EXECUTABLE name extradeps)
	ADD_EXECUTABLE(${name} ${name}.cpp)
	TARGET_LINK_LIBRARIES(${name} ${MRPT_LIBS}) # Add the required libraries for linking:
	
	# -------------------------------------------------------------------------
	# This part can be removed if you are compiling this program outside of 
	#  the MRPT tree:
	# -------------------------------------------------------------------------
	IF("${CMAKE_PROJECT_NAME}" STREQUAL "MRPT") # Fails if build outside of MRPT project.
		DeclareAppDependencies(${name} mrpt-srba mrpt-gui ${extradeps}) # Dependencies
	ENDIF("${CMAKE_PROJECT_NAME}" STREQUAL "MRPT")
	# -------------------------------------------------------------------------
ENDMACRO(DEFINE_TUTORIAL_EXECUTABLE)


# --------------------------------------------------------------------
#  List of tutorials/examples:
# --------------------------------------------------------------------
DEFINE_TUTORIAL_EXECUTABLE("tutorial-srba-monocular-se3" "")
DEFINE_TUTORIAL_EXECUTABLE("tutorial-srba-stereo-se2" "")
DEFINE_TUTORIAL_EXECUTABLE("tutorial-srba-stereo-se3" "")
DEFINE_TUTORIAL_EXECUTABLE("tutorial-srba-cartesian2d-se2" "")
DEFINE_TUTORIAL_EXECUTABLE("tutorial-srba-cartesian3d-se3" "")
DEFINE_TUTORIAL_EXECUTABLE("tutorial-srba-range-bearing-se2" "")
DEFINE_TUTORIAL_EXECUTABLE("tutorial-srba-range-bearing-se3" "")
DEFINE_TUTORIAL_EXECUTABLE("tutorial-srba-relative-graph-slam-se2" "")
DEFINE_TUTORIAL_EXECUTABLE("tutorial-srba-relative-graph-slam-se3" "")
DEFINE_TUTORIAL_EXECUTABLE("tutorial-srba-remove-observation" "")
DEFINE_TUTORIAL_EXECUTABLE("tutorial-srba-how-to-recover-global-map" "mrpt-graphslam")
