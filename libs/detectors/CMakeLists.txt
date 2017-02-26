IF(CMAKE_MRPT_HAS_OPENCV)
	INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS} ${OPENCV_INCLUDEDIR})
ENDIF()

#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib(
	# Lib name
	detectors
	# Deps:
	mrpt-vision
	mrpt-maps
	mrpt-gui
	mrpt-slam
	# And deps of the deps:
	mrpt-base
	mrpt-opengl
	mrpt-obs
	mrpt-graphs
	)

IF(BUILD_mrpt-detectors)
	IF(CMAKE_MRPT_HAS_OPENCV)
		target_link_libraries(mrpt-detectors ${OpenCV_LIBRARIES})
	ENDIF()

ENDIF()
