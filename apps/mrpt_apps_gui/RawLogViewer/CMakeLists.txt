#-----------------------------------------------------------------------------------------------
# CMake file for MRPT app
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(RawLogViewer)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_slam REQUIRED)
find_package(mrpt_gui REQUIRED)
find_package(mrpt_topography REQUIRED)
find_package(wxWidgets REQUIRED COMPONENTS gl core base adv)

# Define the executable target:
mrpt_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES
    CFormEdit.cpp
    CFormEdit.h
    CFormMotionModel.cpp
    CFormMotionModel.h
    CFormPlayVideo.cpp
    CFormPlayVideo.h
    CFormRawMap.cpp
    CFormRawMap.h
    CScanMatching.cpp
    CScanMatching.h
    COdometryParams.cpp
    COdometryParams.h
    CScanAnimation.cpp
    CScanAnimation.h
    ViewOptions3DPoints.cpp
    ViewOptions3DPoints.h
    xRawLogViewerApp.cpp
    xRawLogViewerApp.h
    xRawLogViewerMain.cpp
    xRawLogViewerMain.h
    ../wx-common/MyGLCanvas.cpp
    ../wx-common/MyGLCanvas.h
    CFormChangeSensorPositions.cpp
    CFormChangeSensorPositions.h
    main_gps_ops.cpp
    main_images_ops.cpp
    main_imports_exports.cpp
    main_artwork.cpp
    main_convert_ops.cpp
    main_show_selected_object.cpp
    main_timeline.cpp
    CRawlogTreeView.cpp
    CRawlogTreeView.h
    CFormBatchSensorPose.cpp
    CFormBatchSensorPose.h
    CIniEditor.cpp
    CIniEditor.h
    ../wxThings/src/toggle.cpp
  LINK_LIBRARIES
    mrpt::mrpt_slam
    mrpt::mrpt_gui
    mrpt::mrpt_topography
    wxWidgets::wxWidgets
)

target_include_directories(${PROJECT_NAME} PRIVATE ../wxThings/ ../wx-common/)

