#-----------------------------------------------------------------------------------------------
# CMake file for the MRPT example:  /hwdrivers_gps_example
#
#  Run with "ccmake ." at the root directory, or use it as a template for
#   starting your own programs
#-----------------------------------------------------------------------------------------------
set(sampleName hwdrivers_gps_example)
project(EXAMPLE_${sampleName})

cmake_minimum_required(VERSION 3.1)
cmake_policy(SET CMP0003 NEW)  # Required by CMake 2.7+

# ---------------------------------------------------------------------------
# Set the output directory of each example to its corresponding subdirectory
#  in the binary tree:
# ---------------------------------------------------------------------------
set(EXECUTABLE_OUTPUT_PATH ".")

# The list of "libs" which can be included can be found in:
#  https://www.mrpt.org/Libraries
# Add the top-level dependencies only.
# --------------------------------------------------------------------------
foreach(dep hwdrivers;gui)
	# if not building from inside MRPT source tree, find it as a cmake
	# imported project:
	if (NOT TARGET mrpt-${dep})
		find_package(mrpt-${dep} REQUIRED)
	endif()
endforeach()

# Define the executable target:
add_executable(${sampleName} test.cpp  )

if(TARGET examples)
	add_dependencies(examples ${sampleName})
endif()

set_target_properties(
	${sampleName}
	PROPERTIES
	PROJECT_LABEL "(EXAMPLE) ${sampleName}")

# Add special defines needed by this example, if any:
set(MY_DEFS )
if(MY_DEFS) # If not empty
	add_definitions("-D${MY_DEFS}")
endif()

# Add the required libraries for linking:
foreach(dep hwdrivers;gui)
	target_link_libraries(${sampleName} mrpt-${dep})
endforeach()

# Set optimized building:
if((${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang" OR CMAKE_COMPILER_IS_GNUCXX) AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
	add_compile_options(-O3)
endif()

# This part can be removed if you are compiling this program outside of
#  the MRPT tree:
if(DEFINED MRPT_LIBS_ROOT) # Fails if build outside of MRPT project.
	DeclareAppDependencies(${sampleName} mrpt-hwdrivers;mrpt-gui) # Dependencies
endif()
