if(CMAKE_MRPT_HAS_OPENCV)
	include_directories(${OpenCV_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS} ${OPENCV_INCLUDEDIR})
endif()

define_mrpt_lib(
	img 		# Lib name
	# Dependencies:
	mrpt-core
	mrpt-serialization
	mrpt-io
	mrpt-math
	mrpt-config
	)

if(BUILD_mrpt-img)
	# Ignore precompiled headers in some sources:
	if(MRPT_ENABLE_PRECOMPILED_HDRS)
		if (MSVC)
			set_source_files_properties(
				${img/jpeglib_FILES}
				PROPERTIES COMPILE_FLAGS "/Y-"
				)
		endif ()
	endif()

	if(CMAKE_MRPT_HAS_OPENCV)
		target_link_libraries(mrpt-img PRIVATE ${OpenCV_LIBRARIES})
	endif()
	if(CMAKE_MRPT_HAS_JPEG)
		if(NOT CMAKE_MRPT_HAS_JPEG_SYSTEM)
			add_dependencies(mrpt-img JPEG)
		endif()
		target_link_libraries(mrpt-img PRIVATE ${JPEG_LIBRARIES})
	endif()

endif()
