INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

# ----------------------------------------------------------------------------
#  Auxiliary static library: wxThings
#   Used in some apps/* projects.
# ----------------------------------------------------------------------------

PROJECT(wxThings)

INCLUDE_DIRECTORIES("${MRPT_SOURCE_DIR}/otherlibs/wxThings/")

ADD_LIBRARY(mrptwxthings
	STATIC
	src/toggle.cpp    # fancy button
	)

set_target_properties(mrptwxthings PROPERTIES FOLDER "3rd party")
