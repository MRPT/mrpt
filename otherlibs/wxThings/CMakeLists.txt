include(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

# ----------------------------------------------------------------------------
#  Auxiliary static library: wxThings
#   Used in some apps/* projects.
# ----------------------------------------------------------------------------

project(wxThings)

add_library(mrptwxthings
	STATIC
	src/toggle.cpp    # fancy button
	)

set_target_properties(mrptwxthings PROPERTIES FOLDER "3rd party")

target_include_directories(
	mrptwxthings
	PUBLIC
	"${MRPT_SOURCE_DIR}/otherlibs/wxThings/"
	)
target_link_libraries(mrptwxthings PRIVATE imp_wxwidgets)
