INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

# ----------------------------------------------------------------------------
#  Auxiliary static library: gtest (Google test framework)
#   Used in mrpt-* libraries for Unit Testing
# ----------------------------------------------------------------------------

PROJECT(mrptgtest)

INCLUDE_DIRECTORIES("${MRPT_SOURCE_DIR}/otherlibs/gtest-1.5.0/fused-src/")

ADD_LIBRARY(mrptgtest 
	STATIC
	fused-src/gtest/gtest-all.cc 
	)

IF(CMAKE_COMPILER_IS_GNUCXX AND UNIX)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
ENDIF(CMAKE_COMPILER_IS_GNUCXX AND UNIX)

SET_TARGET_PROPERTIES(mrptgtest  PROPERTIES PROJECT_LABEL "(TEST) mrptgtest")
