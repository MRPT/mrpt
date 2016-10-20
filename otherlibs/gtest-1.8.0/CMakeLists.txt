INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

# ----------------------------------------------------------------------------
#  Auxiliary static library: gtest (Google test framework)
#   Used in mrpt-* libraries for Unit Testing
# ----------------------------------------------------------------------------

PROJECT(mrptgtest)

# Fix a "bug" in VS11 (MSVC 2012):
if(MSVC)
	add_definitions(-D_VARIADIC_MAX=10)
endif(MSVC)

INCLUDE_DIRECTORIES("include")
INCLUDE_DIRECTORIES(".")

ADD_LIBRARY(mrptgtest 
	STATIC
	src/gtest-all.cc 
	)

IF(CMAKE_COMPILER_IS_GNUCXX AND UNIX)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
ENDIF(CMAKE_COMPILER_IS_GNUCXX AND UNIX)


if(ENABLE_SOLUTION_FOLDERS)
	set_target_properties(mrptgtest PROPERTIES FOLDER "3rd party")
else(ENABLE_SOLUTION_FOLDERS)
	SET_TARGET_PROPERTIES(mrptgtest  PROPERTIES PROJECT_LABEL "(TEST) mrptgtest")
endif(ENABLE_SOLUTION_FOLDERS)

