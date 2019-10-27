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

mrpt_lib_target_requires_cpp17(mrptgtest)

if (UNIX AND (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang" OR CMAKE_COMPILER_IS_GNUCXX))
	target_compile_options(mrptgtest PRIVATE "-fPIC")
endif()



set_target_properties(mrptgtest PROPERTIES FOLDER "3rd party")
