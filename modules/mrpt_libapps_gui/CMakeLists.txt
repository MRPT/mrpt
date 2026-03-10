cmake_minimum_required(VERSION 3.16)
project(mrpt_libapps_gui LANGUAGES C CXX)

find_package(mrpt_common REQUIRED)
find_package(mrpt_libapps_cli REQUIRED)
find_package(mrpt_gui REQUIRED)
find_package(CLI11 REQUIRED)
find_package(wxWidgets COMPONENTS core base gl adv QUIET)
if(wxWidgets_FOUND)
  include(${wxWidgets_USE_FILE})
endif()

# define lib:
set(LIB_SOURCES
  src/registerAllClasses.cpp
  src/CameraSelectionGUI.cpp
  src/ICP_SLAM_App.cpp
  src/KFSLAMApp.cpp
  src/MonteCarloLocalization_App.cpp
  src/RBPF_SLAM_App.cpp
  # Tests:
  tests/ICP_SLAM_App_unittest.cpp
  tests/KFSLAMApp_unittest.cpp
  tests/MonteCarloLocalization_App_unittest.cpp
  tests/RBPF_SLAM_App_unittest.cpp
)

set(LIB_PUBLIC_HEADERS
  include/mrpt/apps_gui/CameraSelectionGUI.h
  include/mrpt/apps_gui/CRawlogProcessor.h
  include/mrpt/apps_gui/ICP_SLAM_App.h
  include/mrpt/apps_gui/KFSLAMApp.h
  include/mrpt/apps_gui/MonteCarloLocalization_App.h
  include/mrpt/apps_gui/RBPF_SLAM_App.h
  include/mrpt/apps_gui/RawlogEditApp.h
  include/mrpt/apps_gui/registerAllClasses.h
)

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SOURCES} ${LIB_PUBLIC_HEADERS}
  PUBLIC_LINK_LIBRARIES
    mrpt::mrpt_libapps_cli
    mrpt::mrpt_gui
  CMAKE_DEPENDENCIES
    mrpt_libapps_cli
    mrpt_gui
)

target_link_libraries(${PROJECT_NAME} PRIVATE Threads::Threads CLI11::CLI11)
if(wxWidgets_FOUND)
  target_link_libraries(${PROJECT_NAME} PRIVATE ${wxWidgets_LIBRARIES})
endif()
