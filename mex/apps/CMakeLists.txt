# ----------------------------------------------------------------------------
# CMake file. Run with "cmake ." at the root directory.
# ----------------------------------------------------------------------------
project(ALL_MEX_APPS)

include(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

if (CMAKE_MRPT_HAS_MATLAB)

# Enumerate all subdirs and process them:
list_subdirectories(LIST_MEX_APP_DIRS "${MRPT_SOURCE_DIR}/mex/apps")

foreach(dir ${LIST_MEX_APP_DIRS})
	set(VAR_NAME "BUILD_MEX_${dir}")
	set(${VAR_NAME} ON CACHE BOOL "Build the app ${dir}")
	mark_as_advanced(${VAR_NAME})

	if (${VAR_NAME})
	add_subdirectory(${dir})
	endif (${VAR_NAME})
endforeach(dir)

endif (CMAKE_MRPT_HAS_MATLAB)
