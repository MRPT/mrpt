#-----------------------------------------------------------------------------------------------
# CMake file for MRPT app
#-----------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)

project(robot-map-gui)

# Basic MRPT infrastructure
find_package(mrpt_common REQUIRED)

# Dependencies
find_package(mrpt_slam REQUIRED)
find_package(mrpt_gui REQUIRED)
find_package(mrpt_opengl REQUIRED)
find_package(Qt5 COMPONENTS Widgets OpenGL REQUIRED)
find_package(OpenGL REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTORCC ON)

set(RESOURCES resources/resources.qrc)

set(CPP_FILES
	robot-map-gui_main.cpp
	CUndoManager.cpp
	CDocument.cpp
	gui/CMainWindow.cpp
	gui/glWidget/CGLWidget.cpp
	gui/glWidget/CRobotPose.cpp
	gui/glWidget/CViewerContainer.cpp
	gui/glWidget/CPoseDirection.cpp
	gui/observationTree/CObservationTree.cpp
	gui/observationTree/CObservationTreeModel.cpp
	gui/observationTree/CNode.cpp
	gui/observationTree/CRootNode.cpp
	gui/observationTree/CPosesNode.cpp
	gui/observationTree/CPairNode.cpp
	gui/observationTree/CObservationsNode.cpp
	gui/observationTree/CBaseObservationNode.cpp
	gui/observationTree/CRangeScanNode.cpp
	gui/observationTree/CObservationImageNode.cpp
	gui/observationTree/CObservationStereoImageNode.cpp
	gui/configWidget/CConfigWidget.cpp
	gui/configWidget/CSelectType.cpp
	gui/configWidget/CBaseConfig.cpp
	gui/configWidget/CPointsConfig.cpp
	gui/configWidget/CBeaconConfig.cpp
	gui/configWidget/COccupancyConfig.cpp
	gui/configWidget/CGasGridConfig.cpp
	gui/configWidget/CLandmarksConfig.cpp
	gui/configWidget/CGeneralConfig.cpp
	TypeOfConfig.h
)

set(FILES_TO_MOC
	gui/CMainWindow.h
	gui/glWidget/CGLWidget.h
	gui/configWidget/CConfigWidget.h
	gui/observationTree/CObservationTree.h
	gui/glWidget/CViewerContainer.h
	gui/glWidget/CPoseDirection.h
	gui/configWidget/CGeneralConfig.h
)

set(FILES_TO_UIC
	gui/CMainWindow.ui
	gui/configWidget/CConfigWidget.ui
	gui/configWidget/CSelectType.ui
	gui/configWidget/CPointsConfig.ui
	gui/configWidget/COccupancyConfig.ui
	gui/configWidget/CBeaconConfig.ui
	gui/configWidget/CGasGridConfig.ui
	gui/configWidget/CLandmarksConfig.ui
	gui/configWidget/CGeneralConfig.ui
	gui/glWidget/CViewerContainer.ui
	gui/glWidget/CPoseDirection.ui
)

qt5_wrap_cpp(moc_cpps ${FILES_TO_MOC})
qt5_wrap_ui(uic_cpps ${FILES_TO_UIC})

# Define the executable target:
mrpt_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES
    ${CPP_FILES}
    ${moc_cpps}
    ${uic_cpps}
    ${RESOURCES}
  LINK_LIBRARIES
    mrpt::mrpt_slam
    mrpt::mrpt_gui
    mrpt::mrpt_opengl
    Qt5::Widgets
    Qt5::OpenGL
    OpenGL::GL
)
