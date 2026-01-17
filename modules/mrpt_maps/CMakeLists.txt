
cmake_minimum_required(VERSION 3.16)
project(mrpt_maps LANGUAGES C CXX)

find_package(mrpt_obs REQUIRED)
find_package(mrpt_graphs REQUIRED)

include(./cmake/script_gridmap_options.cmake REQUIRED) # Gridmap options
include(./cmake/script_octomap.cmake REQUIRED)     # Check for the octomap library

# define lib:
set(LIB_SOURCES
  src/CGridMapBase.cpp
  src/CSimpleMap.cpp
)

set(LIB_PUBLIC_HEADERS
  include/mrpt/maps/CGridMapBase.h
  include/mrpt/maps/CSimpleMap.h
)

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SOURCES} ${LIB_PUBLIC_HEADERS}
  PUBLIC_LINK_LIBRARIES
    mrpt::mrpt_obs
    mrpt::mrpt_graphs
  CMAKE_DEPENDENCIES
    mrpt_obs
    mrpt_graphs
)

if(BUILD_mrpt-maps)
	if (CMAKE_MRPT_HAS_OCTOMAP)
		if (NOT "${OCTOMAP_LIBRARY_DIRS}" STREQUAL "")
			link_directories("${OCTOMAP_LIBRARY_DIRS}")
		endif()
		if(TARGET "EP_octomap")
			add_dependencies(${PROJECT_NAME} EP_octomap)
		endif()
		target_link_libraries(${PROJECT_NAME} PRIVATE ${OCTOMAP_LIBRARIES})
	endif()

	target_link_libraries(${PROJECT_NAME} PRIVATE ${MRPT_OPENGL_LIBS})

endif()
