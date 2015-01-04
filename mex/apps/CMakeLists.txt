# ----------------------------------------------------------------------------
# CMake file. Run with "cmake ." at the root directory.
# ----------------------------------------------------------------------------
project(ALL_MEX_APPS)

INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

IF (CMAKE_MRPT_HAS_MATLAB)

# Enumerate all subdirs and process them:
list_subdirectories(LIST_MEX_APP_DIRS "${MRPT_SOURCE_DIR}/mex/apps")

foreach(dir ${LIST_MEX_APP_DIRS})
	SET(VAR_NAME "BUILD_MEX_${dir}")
	SET(${VAR_NAME} ON CACHE BOOL "Build the app ${dir}")
	MARK_AS_ADVANCED(${VAR_NAME})

	if (${VAR_NAME})
	add_subdirectory(${dir})
	endif (${VAR_NAME})
endforeach(dir)

ENDIF (CMAKE_MRPT_HAS_MATLAB)
