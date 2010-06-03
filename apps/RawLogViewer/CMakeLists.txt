INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT application:  RawLogViewer
#
#  Run with "cmake ." at the root directory
#
#  October 2007, Jose Luis Blanco <jlblanco@ctima.uma.es>
#-----------------------------------------------------------------
PROJECT(RawLogViewer)

IF(wxWidgets_FOUND)

	# ---------------------------------------------
	# TARGET:
	# ---------------------------------------------
	SET(SRCS CAboutBox.cpp 	CAboutBox.h
	 CFormEdit.cpp	CFormEdit.h
	 CFormMotionModel.cpp	CFormMotionModel.h
	 CFormPlayVideo.cpp		CFormPlayVideo.h
	 CFormRawMap.cpp		CFormRawMap.h
	 CScanMatching.cpp		CScanMatching.h
	 CFormChangeSensorPositions.cpp CFormChangeSensorPositions.h
	 COdometryParams.cpp 		COdometryParams.h
	 CScanAnimation.cpp			CScanAnimation.h
	 resource.h
	 xRawLogViewerApp.cpp	xRawLogViewerApp.h
	 xRawLogViewerMain.cpp	xRawLogViewerMain.h
	 MyGLCanvas.cpp         MyGLCanvas.h
	 main_gps_ops.cpp
	 main_images_ops.cpp
	 main_imports_exports.cpp
	 main_vodometry.cpp
	 main_artwork.cpp
	 main_convert_ops.cpp
	 CRawlogTreeView.cpp	CRawlogTreeView.h
	 CFormBatchSensorPose.cpp  CFormBatchSensorPose.h
	 CIniEditor.cpp 		CIniEditor.h
	 ../wx-common/CMyRedirector.h
	 )
	 
	IF(WIN32)
		SET(SRCS ${SRCS} resource.rc)
	ENDIF(WIN32)

	# Define the executable target:
	ADD_EXECUTABLE(RawLogViewer WIN32 ${SRCS})

	SET(TMP_TARGET_NAME "RawLogViewer")


	SET_TARGET_PROPERTIES(${TMP_TARGET_NAME} PROPERTIES PROJECT_LABEL "(APP) ${TMP_TARGET_NAME}")

	# Add the required libraries for linking:
	TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${wxWidgets_LIBRARIES} )
	TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${MRPT_LINKER_LIBS})

	# Dependencies on MRPT libraries:
	#  Just mention the top-level dependency, the rest will be detected automatically, 
	#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
	DeclareAppDependencies(${TMP_TARGET_NAME} mrpt-slam mrpt-gui mrpt-topography)
	
	IF(BUILD_STEREOSLAM)
		ADD_DEFINITIONS(-DRAWLOGVIEWER_HAS_STEREOSLAM)
		DeclareAppDependencies(${TMP_TARGET_NAME} mrpt-stereoslam)
	ENDIF(BUILD_STEREOSLAM)	

	# ----------------------------------------------------------------------------
	#   				INSTALL
	# ----------------------------------------------------------------------------
	INSTALL(TARGETS RawLogViewer
	    RUNTIME DESTINATION ${mrpt_apps_INSTALL_PREFIX}bin
	    LIBRARY DESTINATION ${mrpt_apps_INSTALL_PREFIX}lib${LIB_SUFFIX}
	    ARCHIVE DESTINATION ${mrpt_apps_INSTALL_PREFIX}lib${LIB_SUFFIX})


ELSE(wxWidgets_FOUND)
	# ----------------------------------------------------------------------------------
	# For convenience. When we cannot continue, inform the user
	# ----------------------------------------------------------------------------------
    MESSAGE(STATUS "wxWidgets not found!: **This target will not be included in the Makefiles**")
ENDIF(wxWidgets_FOUND)


