include(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

# ----------------------------------------------------------------------------
#  Auxiliary static library: gtest (Google test framework)
#   Used in mrpt-* libraries for Unit Testing
# ----------------------------------------------------------------------------

project(mrptgtest)

# Fix a "bug" in VS11 (MSVC 2012):
if(MSVC)
	add_definitions(-D_VARIADIC_MAX=10)
endif(MSVC)

include_directories("include")
include_directories(".")

add_library(mrptgtest
	STATIC
	src/gtest-all.cc
	)

if(CMAKE_COMPILER_IS_GNUCXX AND UNIX)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
endif(CMAKE_COMPILER_IS_GNUCXX AND UNIX)


set_target_properties(mrptgtest PROPERTIES FOLDER "3rd party")
