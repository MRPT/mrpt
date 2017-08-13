
INCLUDE(../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

IF (CMAKE_MRPT_HAS_WXWIDGETS)
	add_subdirectory(wxThings)
ENDIF (CMAKE_MRPT_HAS_WXWIDGETS)

