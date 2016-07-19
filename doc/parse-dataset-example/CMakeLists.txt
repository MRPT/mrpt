PROJECT(parse_dataset_example)

CMAKE_MINIMUM_REQUIRED(VERSION 2.4)
if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)  # Required by CMake 2.7+
      cmake_policy(SET CMP0005 NEW)
endif()

# --------------------------------------------------------------------------
#   The list of "libs" which can be included can be found in:
#     http://www.mrpt.org/Libraries
#
#   The dependencies of a library are automatically added, so you only
#    need to specify the top-most libraries your code depend on.
# --------------------------------------------------------------------------
FIND_PACKAGE( MRPT REQUIRED base obs maps topography gui)

# Declare the target (an executable)
ADD_EXECUTABLE(parse_dataset_example
	main.cpp
	)
TARGET_LINK_LIBRARIES(parse_dataset_example ${MRPT_LIBS})

# Set optimized building:
IF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")
ENDIF()
