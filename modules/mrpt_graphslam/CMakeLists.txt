cmake_minimum_required(VERSION 3.16)
project(mrpt_graphslam LANGUAGES C CXX)

find_package(mrpt_common REQUIRED)
find_package(mrpt_slam REQUIRED)
find_package(mrpt_gui REQUIRED)

# define lib:
set(LIB_SOURCES
  src/TSlidingWindow.cpp
  src/CWindowObserver.cpp
  src/CEdgeCounter.cpp
  # Tests:
  tests/graph_slam_levmarq_unittest.cpp
)

set(LIB_PUBLIC_HEADERS
  include/mrpt/graphslam/CGraphSlamEngine.h
  include/mrpt/graphslam/CGraphSlamEngine_impl.h
  include/mrpt/graphslam/ERD/CEmptyERD.h
  include/mrpt/graphslam/ERD/CICPCriteriaERD.h
  include/mrpt/graphslam/ERD/CICPCriteriaERD_impl.h
  include/mrpt/graphslam/ERD/CLoopCloserERD.h
  include/mrpt/graphslam/ERD/CLoopCloserERD_impl.h
  include/mrpt/graphslam/GSO/CEmptyGSO.h
  include/mrpt/graphslam/GSO/CLevMarqGSO.h
  include/mrpt/graphslam/GSO/CLevMarqGSO_impl.h
  include/mrpt/graphslam/NRD/CEmptyNRD.h
  include/mrpt/graphslam/NRD/CFixedIntervalsNRD.h
  include/mrpt/graphslam/NRD/CFixedIntervalsNRD_impl.h
  include/mrpt/graphslam/NRD/CICPCriteriaNRD.h
  include/mrpt/graphslam/NRD/CICPCriteriaNRD_impl.h
  include/mrpt/graphslam/apps_related/CGraphSlamHandler.h
  include/mrpt/graphslam/apps_related/CGraphSlamHandler_impl.h
  include/mrpt/graphslam/apps_related/TUserOptionsChecker.h
  include/mrpt/graphslam/apps_related/TUserOptionsChecker_impl.h
  include/mrpt/graphslam/interfaces/CEdgeRegistrationDecider.h
  include/mrpt/graphslam/interfaces/CEdgeRegistrationDecider_impl.h
  include/mrpt/graphslam/interfaces/CGraphSlamOptimizer.h
  include/mrpt/graphslam/interfaces/CGraphSlamOptimizer_impl.h
  include/mrpt/graphslam/interfaces/CIncrementalNodeRegistrationDecider.h
  include/mrpt/graphslam/interfaces/CIncrementalNodeRegistrationDecider_impl.h
  include/mrpt/graphslam/interfaces/CNodeRegistrationDecider.h
  include/mrpt/graphslam/interfaces/CNodeRegistrationDecider_impl.h
  include/mrpt/graphslam/interfaces/CRangeScanEdgeRegistrationDecider.h
  include/mrpt/graphslam/interfaces/CRangeScanEdgeRegistrationDecider_impl.h
  include/mrpt/graphslam/interfaces/CRegistrationDeciderOrOptimizer.h
  include/mrpt/graphslam/interfaces/CRegistrationDeciderOrOptimizer_impl.h
  include/mrpt/graphslam/levmarq.h
  include/mrpt/graphslam/levmarq_impl.h
  include/mrpt/graphslam/misc/CEdgeCounter.h
  include/mrpt/graphslam/misc/CRangeScanOps.h
  include/mrpt/graphslam/misc/CRangeScanOps_impl.h
  include/mrpt/graphslam/misc/CWindowManager.h
  include/mrpt/graphslam/misc/CWindowObserver.h
  include/mrpt/graphslam/misc/TNodeProps.h
  include/mrpt/graphslam/misc/TSlidingWindow.h
  include/mrpt/graphslam/misc/TUncertaintyPath.h
  include/mrpt/graphslam/misc/TUncertaintyPath_impl.h
  include/mrpt/graphslam/registerAllClasses.h
  include/mrpt/graphslam/types.h
)

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SOURCES} ${LIB_PUBLIC_HEADERS}
  PUBLIC_LINK_LIBRARIES
    mrpt::mrpt_slam
    mrpt::mrpt_gui
  CMAKE_DEPENDENCIES
    mrpt_slam
    mrpt_gui
)
