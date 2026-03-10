

#-----------------------------------------------------------------------------------------------
# CMake file for MRPT app
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(mrpt-performance)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_slam REQUIRED)
find_package(mrpt_gui REQUIRED)
find_package(mrpt_tfest REQUIRED)
find_package(mrpt_graphs REQUIRED)
find_package(mrpt_graphslam REQUIRED)
find_package(mrpt_img REQUIRED)
find_package(CLI11 REQUIRED)

# Compile definitions
add_compile_definitions(MRPT_DATASET_DIR="${MRPT_SOURCE_DIR}/share/mrpt/datasets")
add_compile_definitions(MRPT_DOC_PERF_DIR=".")

# Define the executable target:
mrpt_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES
    perf-main.cpp
    common.h
    run_build_tables.h
    perf-graph.cpp
    perf-graphslam.cpp
    perf-gridmaps.cpp
    perf-gridmap3D.cpp
    perf-icp.cpp
    perf-images.cpp
    perf-math.cpp
    perf-matrix1.cpp
    perf-matrix2.cpp
    perf-pointmaps.cpp
    perf-poses.cpp
    perf-pose-interp.cpp
    perf-octomap.cpp
    perf-random.cpp
    perf-scan_matching.cpp
    perf-CObservation3DRangeScan.cpp
    perf-atan2lut.cpp
    perf-strings.cpp
    perf-system.cpp
    perf-yaml.cpp
  LINK_LIBRARIES 
    mrpt::mrpt_slam
    mrpt::mrpt_gui
    mrpt::mrpt_tfest
    mrpt::mrpt_graphs
    mrpt::mrpt_graphslam
    mrpt::mrpt_img
    CLI11::CLI11
)
