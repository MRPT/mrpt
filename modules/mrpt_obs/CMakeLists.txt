# ------------------------------------------------------------------------------
#        Mobile Robot Programming Toolkit (MRPT)
#
# Copyright (c) 2005-2026, Jose Luis Blanco-Claraco, contributors (see Git history)
# All rights reserved.
# Released under BSD-3 license. See LICENSE file
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.16)

# Tell CMake we'll use C++ for use in its tests/flags
project(mrpt_obs LANGUAGES C CXX)

# MRPT CMake scripts: "mrpt_xxx()"
find_package(mrpt_common REQUIRED)
find_package(mrpt_viz REQUIRED)
find_package(mrpt_tfest REQUIRED)

include(./cmake/script_tinyxml2.cmake REQUIRED)  # tinyxml2 lib

# define lib:
set(LIB_SOURCES
  src/CActionCollection.cpp
  src/CAction.cpp
  src/CActionRobotMovement2D.cpp
  src/CActionRobotMovement3D.cpp
  src/carmen_log_tools.cpp
  src/CMetricMap.cpp
  src/CObservation2DRangeScan.cpp
  src/CObservation2DRangeScanWithUncertainty.cpp
  src/CObservation3DRangeScan.cpp
  src/CObservation3DScene.cpp
  src/CObservation6DFeatures.cpp
  src/CObservationBatteryState.cpp
  src/CObservationBeaconRanges.cpp
  src/CObservationBearingRange.cpp
  src/CObservationCANBusJ1939.cpp
  src/CObservationComment.cpp
  src/CObservation.cpp
  src/CObservationGasSensors.cpp
  src/CObservationGPS.cpp
  src/CObservationImage.cpp
  src/CObservationIMU.cpp
  src/CObservationOdometry.cpp
  src/CObservationRange.cpp
  src/CObservationRawDAQ.cpp
  src/CObservationReflectivity.cpp
  src/CObservationRFID.cpp
  src/CObservationRGBD360.cpp
  src/CObservationRobotPose.cpp
  src/CObservationRotatingScan.cpp
  src/CObservationSkeleton.cpp
  src/CObservationStereoImages.cpp
  src/CObservationStereoImagesFeatures.cpp
  src/CObservationVelodyneScan.cpp
  src/CObservationWindSensor.cpp
  src/CObservationWirelessPower.cpp
  src/CRawlog.cpp
  src/CSensoryFrame.cpp
  src/CSimpleMap.cpp
  src/CSinCosLookUpTableFor2DScans.cpp
  src/format_externals_filename.cpp
  src/gnss_messages_ascii_nmea.cpp
  src/gnss_messages_common.cpp
  src/gnss_messages_novatel.cpp
  src/gnss_messages_topcon.cpp
  src/metric_map_types.cpp
  src/registerAllClasses.cpp
  src/sample_image1.h
  src/sample_image2.h
  src/stock_observations.cpp
  src/TMetricMapInitializer.cpp
  src/TMetricMapTypesRegistry.cpp
  src/TPixelLabelInfo.cpp
  src/VelodyneCalibration.cpp
  # tests ----
  tests/CObservationIMU_unittest.cpp
  tests/CRawlog_unittest.cpp
  tests/CSerializable_unittest.cpp
  tests/CSimpleMap_unittest.cpp
)

set(LIB_PUBLIC_HEADERS
  include/mrpt/maps/TMetricMapTypesRegistry.h
  include/mrpt/maps/CSimpleMap.h
  include/mrpt/maps/metric_map_types.h
  include/mrpt/maps/CMetricMapEvents.h
  include/mrpt/maps/CMetricMap.h
  include/mrpt/maps/TMetricMapInitializer.h
  include/mrpt/obs/CSinCosLookUpTableFor2DScans.h
  include/mrpt/obs/obs_utils.h
  include/mrpt/obs/format_externals_filename.h
  include/mrpt/obs/CObservationRFID.h
  include/mrpt/obs/gnss_messages_ascii_nmea.h
  include/mrpt/obs/CObservationRange.h
  include/mrpt/obs/CAction.h
  include/mrpt/obs/CObservationSkeleton.h
  include/mrpt/obs/CObservationRGBD360.h
  include/mrpt/obs/CObservationRotatingScan.h
  include/mrpt/obs/CObservationGPS.h
  include/mrpt/obs/gnss_messages_novatel.h
  include/mrpt/obs/CObservation6DFeatures.h
  include/mrpt/obs/CSensoryFrame.h
  include/mrpt/obs/VelodyneCalibration.h
  include/mrpt/obs/T3DPointsProjectionParams.h
  include/mrpt/obs/registerAllClasses.h
  include/mrpt/obs/CObservationBatteryState.h
  include/mrpt/obs/CRawlog.h
  include/mrpt/obs/CObservation2DRangeScan.h
  include/mrpt/obs/CActionRobotMovement2D.h
  include/mrpt/obs/CObservation3DRangeScan_project3D_impl.h
  include/mrpt/obs/CObservationCANBusJ1939.h
  include/mrpt/obs/CObservationWirelessPower.h
  include/mrpt/obs/CObservationWindSensor.h
  include/mrpt/obs/CObservation2DRangeScanWithUncertainty.h
  include/mrpt/obs/CObservationVelodyneScan.h
  include/mrpt/obs/T2DScanProperties.h
  include/mrpt/obs/obs_frwds.h
  include/mrpt/obs/CActionCollection.h
  include/mrpt/obs/TRangeImageFilter.h
  include/mrpt/obs/CObservationOdometry.h
  include/mrpt/obs/CObservationBeaconRanges.h
  include/mrpt/obs/CObservation3DScene.h
  include/mrpt/obs/gnss_messages_common.h
  include/mrpt/obs/CObservation3DRangeScan.h
  include/mrpt/obs/CObservationComment.h
  include/mrpt/obs/gnss_messages.h
  include/mrpt/obs/CObservationImage.h
  include/mrpt/obs/CObservationRawDAQ.h
  include/mrpt/obs/CObservationBearingRange.h
  include/mrpt/obs/TPixelLabelInfo.h
  include/mrpt/obs/CActionRobotMovement3D.h
  include/mrpt/obs/CObservationStereoImages.h
  include/mrpt/obs/gnss_messages_topcon.h
  include/mrpt/obs/T3DPointsTo2DScanParams.h
  include/mrpt/obs/gnss_messages_type_list.h
  include/mrpt/obs/CObservationStereoImagesFeatures.h
  include/mrpt/obs/carmen_log_tools.h
  include/mrpt/obs/CObservationGasSensors.h
  include/mrpt/obs/CObservation.h
  include/mrpt/obs/CObservationRobotPose.h
  include/mrpt/obs/stock_observations.h
  include/mrpt/obs/CObservationIMU.h
  include/mrpt/obs/CObservationReflectivity.h
  include/mrpt/obs.h
)

# Built-in libtinyxml2?
if(CMAKE_MRPT_HAS_TINYXML2 AND NOT CMAKE_MRPT_HAS_TINYXML2_SYSTEM)
  file(GLOB TINYXML2_SOURCES
    "${mrpt_obs_BINARY_DIR}/3rdparty/tinyxml2/*.cpp"
    "${mrpt_obs_BINARY_DIR}/3rdparty/tinyxml2/*.h"
  )
  list(APPEND LIB_SOURCES ${TINYXML2_SOURCES})
endif()

# Define library
mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SOURCES} ${LIB_PUBLIC_HEADERS}
  PUBLIC_LINK_LIBRARIES
    mrpt::mrpt_viz
    mrpt::mrpt_tfest
  CMAKE_DEPENDENCIES
    mrpt_viz
    mrpt_tfest
  # Other imported targets:
)

# tinyxml2 as system lib:
if (TARGET imp_tinyxml2)
  target_link_libraries(${PROJECT_NAME} PRIVATE imp_tinyxml2)
endif()
# Built-in libtinyxml2?
if(CMAKE_MRPT_HAS_TINYXML2 AND NOT CMAKE_MRPT_HAS_TINYXML2_SYSTEM)
  list(APPEND obs_EXTRA_SRCS
    "${mrpt_obs_BINARY_DIR}/3rdparty/tinyxml2/*.cpp"
    "${mrpt_obs_BINARY_DIR}/3rdparty/tinyxml2/*.h"
    )
  list(APPEND obs_EXTRA_SRCS_NAME
    "tinyxml2" "tinyxml2"
    )
  include_directories("${mrpt_obs_BINARY_DIR}/3rdparty/tinyxml2/")
endif()

# Python wrapper:
#mrpt_add_python_module(obs
#  python_bindings/${PROJECT_NAME}_py.cpp
#)
