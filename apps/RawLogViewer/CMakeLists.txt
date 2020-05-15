#-----------------------------------------------------------------
# CMake file for the MRPT application:  RawLogViewer
#
#  Run with "cmake ." at the root directory
#
#  October 2007, Jose Luis Blanco <jlblanco@ctima.uma.es>
#-----------------------------------------------------------------
project(RawLogViewer)

mrpt_return_if_not_wxwidgets()
if (NOT CMAKE_MRPT_HAS_OPENGL_GLUT)
	return()
endif()

# ---------------------------------------------
# TARGET:
# ---------------------------------------------
set(SRCS
 CFormEdit.cpp	CFormEdit.h
 CFormMotionModel.cpp	CFormMotionModel.h
 CFormPlayVideo.cpp		CFormPlayVideo.h
 CFormRawMap.cpp		CFormRawMap.h
 CScanMatching.cpp		CScanMatching.h
 CFormChangeSensorPositions.cpp CFormChangeSensorPositions.h
 COdometryParams.cpp 		COdometryParams.h
 CScanAnimation.cpp			CScanAnimation.h
 xRawLogViewerApp.cpp	xRawLogViewerApp.h
 xRawLogViewerMain.cpp	xRawLogViewerMain.h
 MyGLCanvas.cpp         MyGLCanvas.h
 main_gps_ops.cpp
 main_images_ops.cpp
 main_imports_exports.cpp
 main_artwork.cpp
 main_convert_ops.cpp
 main_show_selected_object.cpp
 CRawlogTreeView.cpp	CRawlogTreeView.h
 CFormBatchSensorPose.cpp  CFormBatchSensorPose.h
 CIniEditor.cpp 		CIniEditor.h
 )

if(WIN32)
    set(SRCS ${SRCS} resource.rc)
endif()

# Define the executable target:
add_executable(${PROJECT_NAME} WIN32 ${SRCS} ${MRPT_VERSION_RC_FILE})

# Add the required libraries for linking:
target_link_libraries(${PROJECT_NAME} imp_wxwidgets mrptwxthings)

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${PROJECT_NAME} mrpt::slam mrpt::gui mrpt::topography)
DeclareAppForInstall(${PROJECT_NAME})
AppStartMenuLink(${PROJECT_NAME} "RawLog (dataset) viewer")   # Add link to the Start menu (in Windows Installable packages)

