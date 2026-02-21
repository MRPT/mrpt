cmake_minimum_required(VERSION 3.16)
project(mrpt_slam LANGUAGES C CXX)

find_package(mrpt_common REQUIRED)
find_package(mrpt_vision REQUIRED)
find_package(mrpt_maps REQUIRED)
find_package(mrpt_topography REQUIRED)

# Optional TBB support (used in PF_implementations.h):
find_package(TBB QUIET)
if (TBB_FOUND)
	set(CMAKE_MRPT_HAS_TBB 1)
else()
	set(CMAKE_MRPT_HAS_TBB 0)
endif()

set(LIB_SOURCES
  src/registerAllClasses.cpp
  # slam/
  src/slam/CICP.cpp
  src/slam/CLandmarksMap.cpp
  src/slam/CGridMapAligner.cpp
  src/slam/CIncrementalMapPartitioner.cpp
  src/slam/CMetricMapBuilder.cpp
  src/slam/CMetricMapBuilderICP.cpp
  src/slam/CMetricMapBuilderRBPF.cpp
  src/slam/CMetricMapsAlignmentAlgorithm.cpp
  src/slam/CMonteCarloLocalization2D.cpp
  src/slam/CMonteCarloLocalization3D.cpp
  src/slam/CMultiMetricMapPDF.cpp
  src/slam/CMultiMetricMapPDF_RBPF.cpp
  src/slam/COccupancyGridMapFeatureExtractor.cpp
  src/slam/CRangeBearingKFSLAM.cpp
  src/slam/CRangeBearingKFSLAM2D.cpp
  src/slam/CRejectionSamplingRangeOnlyLocalization.cpp
  src/slam/TKLDParams.cpp
  src/slam/data_association.cpp
  src/slam/observations_overlap.cpp
  src/slam/path_from_rtk_gps.cpp
  # Tests:
  tests/CICP_unittest.cpp
  tests/CIncrementalMapPartitioner_unittest.cpp
  tests/CMonteCarloLocalization2D_unittest.cpp
  tests/data_association_unittest.cpp
  tests/path_from_rtk_gps_unittest.cpp
  tests/se3_ransac_unittest.cpp
)

set(LIB_PUBLIC_HEADERS
  include/mrpt/maps/CLandmark.h
  include/mrpt/maps/CLandmarksMap.h
  include/mrpt/maps/CMultiMetricMapPDF.h
  include/mrpt/slam/CICP.h
  include/mrpt/slam/CGridMapAligner.h
  include/mrpt/slam/CIncrementalMapPartitioner.h
  include/mrpt/slam/CMetricMapBuilder.h
  include/mrpt/slam/CMetricMapBuilderICP.h
  include/mrpt/slam/CMetricMapBuilderRBPF.h
  include/mrpt/slam/CMetricMapsAlignmentAlgorithm.h
  include/mrpt/slam/CMonteCarloLocalization2D.h
  include/mrpt/slam/CMonteCarloLocalization3D.h
  include/mrpt/slam/COccupancyGridMapFeatureExtractor.h
  include/mrpt/slam/CRangeBearingKFSLAM.h
  include/mrpt/slam/CRangeBearingKFSLAM2D.h
  include/mrpt/slam/CRejectionSamplingRangeOnlyLocalization.h
  include/mrpt/slam/PF_implementations.h
  include/mrpt/slam/PF_implementations_data.h
  include/mrpt/slam/TKLDParams.h
  include/mrpt/slam/TMonteCarloLocalizationParams.h
  include/mrpt/slam/data_association.h
  include/mrpt/slam/observations_overlap.h
  include/mrpt/slam/path_from_rtk_gps.h
  include/mrpt/slam/registerAllClasses.h
)

# Ensure downstream consumers can find TBB::tbb (exported as PUBLIC):
if (TBB_FOUND)
	set(EXTRA_CONFIG_CMDS "find_dependency(TBB QUIET)")
endif()

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SOURCES} ${LIB_PUBLIC_HEADERS}
  PUBLIC_LINK_LIBRARIES
    mrpt::mrpt_vision
    mrpt::mrpt_maps
    mrpt::mrpt_topography
  CMAKE_DEPENDENCIES
    mrpt_vision
    mrpt_maps
    mrpt_topography
)

if (TBB_FOUND)
	target_link_libraries(${PROJECT_NAME} PUBLIC TBB::tbb)
endif()
