# ----------------------------------------------------------------------------
# CMake file. Run with "cmake ." at the root directory.
# ----------------------------------------------------------------------------

INCLUDE(../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

# Enumerate all subdirs and process them:
list_subdirectories(LIST_MRPT_APP_DIRS "${MRPT_SOURCE_DIR}/apps")

foreach(dir ${LIST_MRPT_APP_DIRS})
  add_subdirectory(${dir})
endforeach(dir)
