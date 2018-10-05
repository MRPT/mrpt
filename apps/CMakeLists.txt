# ----------------------------------------------------------------------------
# CMake file. Run with "cmake ." at the root directory.
# ----------------------------------------------------------------------------
project(ALL_MRPT_APPS)

include(../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

# Enumerate all subdirs and process them:
list_subdirectories(LIST_MRPT_APP_DIRS "${MRPT_SOURCE_DIR}/apps")

foreach(dir ${LIST_MRPT_APP_DIRS})
	set(VAR_NAME "BUILD_APP_${dir}")
	set(${VAR_NAME} ON CACHE BOOL "Build the app ${dir}")
	mark_as_advanced(FORCE ${VAR_NAME})

	if (${VAR_NAME})
	add_subdirectory(${dir})
	endif (${VAR_NAME})
endforeach(dir)
