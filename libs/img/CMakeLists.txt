IF(CMAKE_MRPT_HAS_OPENCV)
	INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS} ${OPENCV_INCLUDEDIR})
ENDIF()

define_mrpt_lib(
	img 		# Lib name
	# Dependencies:
	mrpt-core
	mrpt-serialization
	mrpt-io
	mrpt-math
	mrpt-config
	)

IF(BUILD_mrpt-img)
	# Ignore precompiled headers in some sources:
	IF(MRPT_ENABLE_PRECOMPILED_HDRS)
		IF (MSVC)
			set_source_files_properties(
				${img/jpeglib_FILES}
				PROPERTIES COMPILE_FLAGS "/Y-"
				)
		ENDIF ()
	ENDIF()

	IF(CMAKE_MRPT_HAS_OPENCV)
		target_link_libraries(mrpt-img PRIVATE ${OpenCV_LIBRARIES})
	ENDIF()
	IF(CMAKE_MRPT_HAS_JPEG)
		IF(NOT CMAKE_MRPT_HAS_JPEG_SYSTEM)
			ADD_DEPENDENCIES(mrpt-img JPEG)
		ENDIF()
		target_link_libraries(mrpt-img PRIVATE ${JPEG_LIBRARIES})
	ENDIF()

ENDIF()
