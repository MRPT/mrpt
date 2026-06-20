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

# toggle.cpp is 3rd-party code — suppress conversion warnings we can't fix
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  set_source_files_properties(src/toggle.cpp PROPERTIES
    COMPILE_FLAGS "-Wno-conversion -Wno-float-conversion -Wno-old-style-cast")
endif()

set_target_properties(mrptwxthings PROPERTIES FOLDER "3rd party")

target_include_directories(
	mrptwxthings
	SYSTEM PUBLIC
	"${MRPT_SOURCE_DIR}/3rdparty/wxThings/"
	)
target_link_libraries(mrptwxthings PRIVATE imp_wxwidgets)

