# ------------------------------------------------------------------------------
#        Mobile Robot Programming Toolkit (MRPT)
#
# Copyright (c) 2005-2026, Jose Luis Blanco-Claraco, contributors (see Git history)
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.16)
project(mrpt_maps LANGUAGES C CXX)

# MRPT CMake scripts: "mrpt_xxx()"
find_package(mrpt_common REQUIRED)
find_package(mrpt_obs REQUIRED)
find_package(mrpt_graphs REQUIRED)

include(./cmake/script_gridmap_options.cmake REQUIRED) # Gridmap options
include(./cmake/script_octomap.cmake REQUIRED)     # Check for the octomap library

# define lib:
set(LIB_SOURCES
  src/registerAllClasses.cpp
  # maps:
  src/maps/CBeacon.cpp
  src/maps/CBeaconMap.cpp
  src/maps/CGasConcentrationGridMap2D.cpp
  src/maps/CGenericPointsMap.cpp
  src/maps/CColouredOctoMap.cpp
  src/maps/CHeightGridMap2D.cpp
  src/maps/CHeightGridMap2D_Base.cpp
  src/maps/CHeightGridMap2D_MRF.cpp
  src/maps/CMultiMetricMap.cpp
  src/maps/COccupancyGridMap2D_common.cpp
  src/maps/COccupancyGridMap2D_getAs.cpp
  src/maps/COccupancyGridMap2D_insert.cpp
  src/maps/COccupancyGridMap2D_io.cpp
  src/maps/COccupancyGridMap2D_likelihood.cpp
  src/maps/COccupancyGridMap2D_simulate.cpp
  src/maps/COccupancyGridMap2D_voronoi.cpp
  src/maps/COccupancyGridMap3D.cpp
  src/maps/COccupancyGridMap3D_insert.cpp
  src/maps/COccupancyGridMap3D_likelihood.cpp
  src/maps/COctoMap.cpp
  src/maps/CPointCloudFilterBase.cpp
  src/maps/CPointCloudFilterByDistance.cpp
  src/maps/CPointsMap.cpp
  src/maps/CRandomFieldGridMap2D.cpp
  src/maps/CRandomFieldGridMap3D.cpp
  src/maps/CReflectivityGridMap2D.cpp
  src/maps/CSimplePointsMap.cpp
  src/maps/CVoxelMap.cpp
  src/maps/CVoxelMapOccupancyBase.cpp
  src/maps/CVoxelMapRGB.cpp
  src/maps/CWirelessPowerGridMap2D.cpp
  # obs:
  src/obs/CObservationPointCloud.cpp
  src/obs/customizable_obs_viz.cpp
  # viz:
  src/viz/CAngularObservationMesh.cpp
  src/viz/CPlanarLaserScan.cpp
  # tests:
  tests/CGasConcentrationGridMap2D_unittest.cpp
  tests/CHeightGridMap2Ds_unittest.cpp
  tests/CLogOddsGridMapLUT_unittest.cpp
  tests/CMultiMetricMap_unittest.cpp
  tests/CObservation3DRangeScan_unittest.cpp
  tests/CObservationRotatingScan_unittest.cpp
  tests/COccupancyGridMap2D_unittest.cpp
  tests/COccupancyGridMap3D_unittest.cpp
  tests/COctoMap_unittest.cpp
  tests/CPointCloudFilterByDistance_unittest.cpp
  tests/CPointsMap_unittest.cpp
  tests/CRandomFieldGridMap3D_unittest.cpp
  tests/serializations_unittest.cpp
)

set(LIB_PUBLIC_HEADERS
  include/mrpt/maps.h
  include/mrpt/maps/CBeacon.h
  include/mrpt/maps/CBeaconMap.h
  include/mrpt/maps/CGasConcentrationGridMap2D.h
  include/mrpt/maps/CColouredOctoMap.h
  include/mrpt/maps/CGenericPointsMap.h
  include/mrpt/maps/CHeightGridMap2D.h
  include/mrpt/maps/CHeightGridMap2D_Base.h
  include/mrpt/maps/CHeightGridMap2D_MRF.h
  include/mrpt/maps/CLogOddsGridMap2D.h
  include/mrpt/maps/CLogOddsGridMap3D.h
  include/mrpt/maps/CLogOddsGridMapLUT.h
  include/mrpt/maps/CMultiMetricMap.h
  include/mrpt/maps/COccupancyGridMap2D.h
  include/mrpt/maps/COccupancyGridMap3D.h
  include/mrpt/maps/COctoMap.h
  include/mrpt/maps/COctoMapBase.h
  include/mrpt/maps/CPointCloudFilterBase.h
  include/mrpt/maps/CPointCloudFilterByDistance.h
  include/mrpt/maps/CPointsMap.h
  include/mrpt/maps/CRandomFieldGridMap2D.h
  include/mrpt/maps/CRandomFieldGridMap3D.h
  include/mrpt/maps/CReflectivityGridMap2D.h
  include/mrpt/maps/CSimplePointsMap.h
  include/mrpt/maps/CVoxelMap.h
  include/mrpt/maps/CVoxelMapBase.h
  include/mrpt/maps/CVoxelMapOccupancyBase.h
  include/mrpt/maps/CVoxelMapRGB.h
  include/mrpt/maps/CWirelessPowerGridMap2D.h
  include/mrpt/maps/logoddscell_traits.h
  include/mrpt/maps/NearestNeighborsCapable.h
  include/mrpt/maps/OccupancyGridCellType.h
  include/mrpt/maps/PCL_adapters.h
  include/mrpt/maps/registerAllClasses.h
  include/mrpt/maps/bonxai/bonxai.hpp
  include/mrpt/maps/bonxai/serialization.hpp
  include/mrpt/obs/CObservationPointCloud.h
  include/mrpt/obs/customizable_obs_viz.h
  include/mrpt/viz/CAngularObservationMesh.h
  include/mrpt/viz/CPlanarLaserScan.h
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

if (CMAKE_MRPT_HAS_OCTOMAP)
	if (NOT "${OCTOMAP_LIBRARY_DIRS}" STREQUAL "")
		link_directories("${OCTOMAP_LIBRARY_DIRS}")
	endif()
	if(TARGET "EP_octomap")
		add_dependencies(${PROJECT_NAME} EP_octomap)
	endif()
	target_link_libraries(${PROJECT_NAME} PRIVATE ${OCTOMAP_LIBRARIES})
endif()

# Python bindings
mrpt_add_python_module(maps
  python_bindings/mrpt_maps_py.cpp
)
