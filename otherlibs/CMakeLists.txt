
INCLUDE(../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

IF (CMAKE_MRPT_HAS_WXWIDGETS)
	add_subdirectory(wxThings)
ENDIF (CMAKE_MRPT_HAS_WXWIDGETS)


# This is not needed to be included here, it's done from the main 
# CMakeLists.txt so it can be switched to a system version, if available.
#add_subdirectory(gtest-X.Y.Z)

# Don't include here: will be included from master CMakeLists.txt with proper parameters.
#add_subdirectory(assimp)

