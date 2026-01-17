
cmake_minimum_required(VERSION 3.16)
project(mrpt_slam LANGUAGES C CXX)

find_package(mrpt_vision REQUIRED)
find_package(mrpt_maps REQUIRED)

include(${MRPT_SOURCE_DIR}/cmakemodules/script_tbb.cmake REQUIRED)         # TBB

# define lib:
set(LIB_SOURCES
  src/CICP.cpp
)

set(LIB_PUBLIC_HEADERS
  include/mrpt/slam/CICP.h
)

if(CMAKE_MRPT_HAS_TBB)
	set(EXTRA_CONFIG_CMDS
		"find_dependency(TBB CONFIG)")
endif()

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SOURCES} ${LIB_PUBLIC_HEADERS}
  PUBLIC_LINK_LIBRARIES
    mrpt::mrpt_vision
    mrpt::mrpt_maps
  CMAKE_DEPENDENCIES
    mrpt_vision
    mrpt_maps
)

if(BUILD_mrpt-slam)

	if (CMAKE_MRPT_HAS_TBB)
		target_link_libraries(${PROJECT_NAME} PUBLIC TBB::tbb)
	endif()
endif()
