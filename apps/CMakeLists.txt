# ----------------------------------------------------------------------------
# CMake file. Run with "cmake ." at the root directory.
# ----------------------------------------------------------------------------

INCLUDE(../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

if(BUILD_MONOSLAM)
#	add_subdirectory(MonoSLAM_demo)
	add_subdirectory(mono-slam)
endif(BUILD_MONOSLAM)

IF(BUILD_APPs_GUIs)
	add_subdirectory(ReactiveNavigationDemo)
	add_subdirectory(navlog-viewer)
	add_subdirectory(prrt-navigator-demo)
	add_subdirectory(RawLogViewer)
	add_subdirectory(3DSceneViewer)
	#add_subdirectory(console2gui)
	add_subdirectory(2d-slam-demo)
	if(EXISTS ${MRPT_SOURCE_DIR}/apps/hmtMapViewer)
		add_subdirectory(hmtMapViewer)
	endif(EXISTS ${MRPT_SOURCE_DIR}/apps/hmtMapViewer)
	add_subdirectory(camera-calib)
ENDIF(BUILD_APPs_GUIs)

IF(BUILD_APPs_SLAM)
	if(EXISTS ${MRPT_SOURCE_DIR}/apps/hmt-slam)
		add_subdirectory(hmt-slam)
	endif(EXISTS ${MRPT_SOURCE_DIR}/apps/hmt-slam)
	if(EXISTS ${MRPT_SOURCE_DIR}/apps/hmt-slam-gui)
		add_subdirectory(hmt-slam-gui)
	endif(EXISTS ${MRPT_SOURCE_DIR}/apps/hmt-slam-gui)
	add_subdirectory(rbpf-slam)
	add_subdirectory(kf-slam)
	add_subdirectory(icp-slam)
ENDIF(BUILD_APPs_SLAM)

IF(BUILD_APPs_SIMUL)
	add_subdirectory(simul-beacons)
	add_subdirectory(simul-landmarks)
	add_subdirectory(simul-gridmap)
	add_subdirectory(GridmapNavSimul)
ENDIF(BUILD_APPs_SIMUL)


add_subdirectory(grid-matching)
add_subdirectory(pf-localization)
add_subdirectory(ro-localization)
add_subdirectory(map-partition)
add_subdirectory(features-matching)
add_subdirectory(mrpt-performance)

# This program requires opencv version 1.1.0:
IF (MRPT_OPENCV_VERSION STRGREATER "1.0.0")
	add_subdirectory(stereo-calib-gui)
ENDIF (MRPT_OPENCV_VERSION STRGREATER "1.0.0")

add_subdirectory(observations2map)
add_subdirectory(rawlog-grabber)

IF (BUILD_STEREOSLAM AND EXISTS "${MRPT_SOURCE_DIR}/apps/vOdometry")
	add_subdirectory(vOdometry)
ENDIF (BUILD_STEREOSLAM AND EXISTS "${MRPT_SOURCE_DIR}/apps/vOdometry")


