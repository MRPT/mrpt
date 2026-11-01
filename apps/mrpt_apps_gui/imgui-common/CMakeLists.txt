#-----------------------------------------------------------------------------------------------
# Internal (not installed) library shared by the ImGui apps: observation panels.
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(mrpt_apps_imgui_common)

find_package(mrpt_common REQUIRED)
find_package(mrpt_maps REQUIRED)
find_package(mrpt_imgui QUIET)
if(NOT mrpt_imgui_FOUND)
  message(STATUS "mrpt_imgui not found (built without GLFW?): skipping ${PROJECT_NAME}")
  return()
endif()

add_library(${PROJECT_NAME} STATIC
  include/mrpt/apps_imgui/ObservationPanels.h
  src/ObservationPanelRegistry.cpp
  src/panel_utils.h
  src/panels_images.cpp
  src/panels_scene.cpp
  src/panels_text.cpp
)
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)
mrpt_set_target_build_options(${PROJECT_NAME} FALSE)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(${PROJECT_NAME} PUBLIC mrpt::mrpt_imgui mrpt::mrpt_maps)
add_library(mrpt_apps::imgui_common ALIAS ${PROJECT_NAME})

mrpt_add_test(
  TARGET test_${PROJECT_NAME}
  SOURCES tests/ObservationPanels_unittest.cpp
  LINK_LIBRARIES ${PROJECT_NAME}
)
