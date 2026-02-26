cmake_minimum_required(VERSION 3.16)
project(mrpt_libapps_cli LANGUAGES C CXX)

find_package(mrpt_common REQUIRED)
find_package(mrpt_hwdrivers REQUIRED)
find_package(mrpt_slam REQUIRED)
find_package(mrpt_topography REQUIRED)
find_package(CLI11 REQUIRED)

# define lib:
set(LIB_SOURCES
  src/registerAllClasses.cpp
  src/DataSourceRawlog.cpp
  src/RawlogEditApp.cpp
  src/RawlogGrabberApp.cpp
  src/rawlog-edit_2d-scans.cpp
  src/rawlog-edit_anemometer.cpp
  src/rawlog-edit_camera-params.cpp
  src/rawlog-edit_cuts.cpp
  src/rawlog-edit_deexternalize.cpp
  src/rawlog-edit_describe.cpp
  src/rawlog-edit_enose.cpp
  src/rawlog-edit_export_txt.cpp
  src/rawlog-edit_externalize.cpp
  src/rawlog-edit_filters.cpp
  src/rawlog-edit_generate-3d-pointclouds.cpp
  src/rawlog-edit_gps.cpp
  src/rawlog-edit_gps_gas.cpp
  src/rawlog-edit_imu.cpp
  src/rawlog-edit_info.cpp
  src/rawlog-edit_list-images.cpp
  src/rawlog-edit_list-poses.cpp
  src/rawlog-edit_list-rangbear.cpp
  src/rawlog-edit_list-timestamps.cpp
  src/rawlog-edit_odometry.cpp
  src/rawlog-edit_rawdaq.cpp
  src/rawlog-edit_remap_timestamps.cpp
  src/rawlog-edit_rename_externals.cpp
  src/rawlog-edit_sensor-poses.cpp
  src/rawlog-edit_stereo-rectify.cpp
  src/rawlog-edit_undistort.cpp
  # Tests:
  tests/RawlogGrabberApp_unittest.cpp
)

set(LIB_PUBLIC_HEADERS
  include/mrpt/apps_cli/BaseAppDataSource.h
  include/mrpt/apps_cli/BaseAppInitializableCLI.h
  include/mrpt/apps_cli/BaseAppInitializableConfig.h
  include/mrpt/apps_cli/CRawlogProcessor.h
  include/mrpt/apps_cli/DataSourceRawlog.h
  include/mrpt/apps_cli/MonteCarloLocalization_App.h
  include/mrpt/apps_cli/RawlogEditApp.h
  include/mrpt/apps_cli/RawlogGrabberApp.h
  include/mrpt/apps_cli/registerAllClasses.h
)

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SOURCES} ${LIB_PUBLIC_HEADERS}
  PUBLIC_LINK_LIBRARIES
    mrpt::mrpt_hwdrivers
    mrpt::mrpt_slam
    mrpt::mrpt_topography
  PRIVATE_LINK_LIBRARIES
    Threads::Threads
    CLI11::CLI11
  CMAKE_DEPENDENCIES
    mrpt_hwdrivers
    mrpt_slam
    mrpt_topography
)
