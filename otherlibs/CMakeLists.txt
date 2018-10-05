
include(../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

if (CMAKE_MRPT_HAS_WXWIDGETS)
	add_subdirectory(wxThings)
endif (CMAKE_MRPT_HAS_WXWIDGETS)

