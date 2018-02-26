# This file is included from the main CMakeLists.txt
#
SET( BUILD_EXAMPLES OFF CACHE BOOL "Build examples?")
IF(BUILD_EXAMPLES)
	SET(CMAKE_MRPT_EXAMPLES_BASE_DIRECTORY "${CMAKE_SOURCE_DIR}/samples/")
	# Fix "\" --> "\\" for windows:
	string(REPLACE "\\" "\\\\" CMAKE_MRPT_EXAMPLES_BASE_DIRECTORY ${CMAKE_MRPT_EXAMPLES_BASE_DIRECTORY})

	#MESSAGE(STATUS "Parsing 'examples_config.h.in'")
	CONFIGURE_FILE("${CMAKE_SOURCE_DIR}/parse-files/examples_config.h.in" "${MRPT_CONFIG_FILE_INCLUDE_DIR}/mrpt/examples_config.h")

	# Generate CMakeLists.txt from the template project file for examples:
	MESSAGE(STATUS "Generating CMakefiles.txt for examples...")

	# ---------------------------------------------------------------
	#  MACRO for samples directories
	# ---------------------------------------------------------------
	MACRO(GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY)
		#TODO: Disable build if BUILD_mrpt_xxx is OFF

		# Convert CMAKE_EXAMPLE_DEPS -> CMAKE_EXAMPLE_DEPS_STRIP
		#          "mrpt-xxx mrpt-yyy" -> "xxx yyy"
		SET(CMAKE_EXAMPLE_DEPS_STRIP "")
		FOREACH(DEP ${CMAKE_EXAMPLE_DEPS})
			# Only for "mrpt-XXX" libs:
			STRING(REGEX REPLACE "mrpt-(.*)" "\\1" STRIP_DEP ${DEP})
			IF(NOT "${STRIP_DEP}" STREQUAL "")
				LIST(APPEND CMAKE_EXAMPLE_DEPS_STRIP ${STRIP_DEP})
			ENDIF(NOT "${STRIP_DEP}" STREQUAL "")
		ENDFOREACH(DEP)

		FOREACH(CMAKE_MRPT_EXAMPLE_NAME ${LIST_EXAMPLES_IN_THIS_DIR})
			#MESSAGE(STATUS "Example: ${CMAKE_MRPT_EXAMPLE_NAME}")
			# Generate project file:
			CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/samples/CMakeLists_template.txt.in ${CMAKE_SOURCE_DIR}/samples/${CMAKE_MRPT_EXAMPLE_NAME}/CMakeLists.txt @ONLY)
			SET(CMAKE_COMMANDS_INCLUDE_EXAMPLE_DIRS_ROOT "${CMAKE_COMMANDS_INCLUDE_EXAMPLE_DIRS_ROOT}\nadd_subdirectory(${CMAKE_MRPT_EXAMPLE_NAME})")
		ENDFOREACH(CMAKE_MRPT_EXAMPLE_NAME)
	ENDMACRO(GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY)

	MACRO(ADD_SAMPLES_DIRECTORY dir)
		SET(CMAKE_COMMANDS_INCLUDE_EXAMPLE_DIRS_ROOT "${CMAKE_COMMANDS_INCLUDE_EXAMPLE_DIRS_ROOT}\nadd_subdirectory(${dir})")
	ENDMACRO()
	# ---------------------------------------------------------------
	#  END OF MACRO for samples directories
	# ---------------------------------------------------------------

	# -----------------------------------------------------------------
	# This loop is generic, do not modify it...
	#  modify the above variable and/or the list_examples.txt files!
	# -----------------------------------------------------------------
	SET(CMAKE_COMMANDS_INCLUDE_EXAMPLE_DIRS_ROOT "")
	#SET(CMAKE_EXAMPLE_LINK_LIBS ${MRPT_LINKER_LIBS}) # use CMake Interface?

	# === Depending on: typemeta ===
	SET(LIST_EXAMPLES_IN_THIS_DIR
		typemeta_TTypeName
		typemeta_StaticString
		typemeta_TEnumType
		)
	SET(CMAKE_EXAMPLE_DEPS mrpt-typemeta)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: rtti ===
	SET(LIST_EXAMPLES_IN_THIS_DIR
		rtti_example1
		)
	SET(CMAKE_EXAMPLE_DEPS mrpt-rtti)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: serialization, io ===
	SET(LIST_EXAMPLES_IN_THIS_DIR
		# ----
		serialization_stl
		serialization_variant_example
		# ----
		io_pipes_example
		io_compress_example
		)
	SET(CMAKE_EXAMPLE_DEPS mrpt-serialization mrpt-io mrpt-poses)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: db ===
	SET(LIST_EXAMPLES_IN_THIS_DIR
		db_example
		)
	SET(CMAKE_EXAMPLE_DEPS mrpt-db mrpt-io mrpt-serialization)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: system ===
	SET(LIST_EXAMPLES_IN_THIS_DIR
		system_datetime_example
		system_directory_explorer_example
		system_file_system_watcher
		system_backtrace_example
		system_dirs_files_manipulation
		system_params_by_name
		)
	SET(CMAKE_EXAMPLE_DEPS mrpt-system mrpt-poses mrpt-io)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: mrpt-math mrpt-random mrpt-gui ===
	SET(LIST_EXAMPLES_IN_THIS_DIR
		math_csparse_example
		math_kmeans_example
		math_matrix_example
		math_optimize_lm_example
		math_slerp_example
		math_spline_interpolation
		math_leastsquares_example
		math_ransac_plane3d_example
		math_ransac_examples
		math_model_search_example
		math_polygon_split
		)
	SET(CMAKE_EXAMPLE_DEPS mrpt-math mrpt-random mrpt-gui)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	SET(LIST_EXAMPLES_IN_THIS_DIR
		math_polyhedron_intersection_example
		)
	SET(CMAKE_EXAMPLE_DEPS mrpt-math mrpt-random mrpt-gui mrpt-maps)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: mrpt-poses ===
	SET(LIST_EXAMPLES_IN_THIS_DIR
		poses_geometry_3D_example
		poses_pdfs_example
		poses_se3_lie_example
		poses_quaternions_example
		poses_sog_merge_example
		poses_unscented_transform_example
		)
	SET(CMAKE_EXAMPLE_DEPS mrpt-poses mrpt-io mrpt-gui)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: mrpt-comms ===
	SET(LIST_EXAMPLES_IN_THIS_DIR
		comms_http_client
		comms_serial_port_example
		comms_socket_example
		comms_ftdi_usb_enumerate_example
		)
	SET(CMAKE_EXAMPLE_DEPS mrpt-comms mrpt-serialization mrpt-poses)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: mrpt-gui, mrpt-img, mrpt-opengl ===
	SET(LIST_EXAMPLES_IN_THIS_DIR
		# -------
		opengl_objects_demo
		opengl_offscreen_render_example
		opengl_octree_render_huge_pointcloud
		opengl_texture_sizes_test
		opengl_textured_triangles_example
		# -------
		img_basic_example
		img_correlation_example
		img_convolution_fft
		img_fft_example
		img_gauss_filtering_example
		# -------
		gui_capture_render_to_img_example
		gui_display3D_example
		gui_display3D_custom_render
		gui_fbo_render_example
		gui_display_plots
		gui_text_fonts_example
		gui_gravity3d_example
		gui_windows_events
		)
	SET(CMAKE_EXAMPLE_DEPS mrpt-gui)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: mrpt-gui, mrpt-hwdrivers ===
	SET(LIST_EXAMPLES_IN_THIS_DIR
		opengl_video_demo
		opengl_video_viewport_demo
		)
	SET(CMAKE_EXAMPLE_DEPS mrpt-gui mrpt-hwdrivers)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: mrpt-gui, mrpt-maps ===
	SET(LIST_EXAMPLES_IN_THIS_DIR
		opengl_ray_trace_example
		)
	SET(CMAKE_EXAMPLE_DEPS mrpt-gui mrpt-maps)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: mrpt-random ===
	SET(LIST_EXAMPLES_IN_THIS_DIR
		random_examples
		)
	SET(CMAKE_EXAMPLE_DEPS mrpt-random mrpt-gui)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: mrpt-bayes, mrpt-obs, mrpt-gui ===
	SET(LIST_EXAMPLES_IN_THIS_DIR
		bayes_tracking_example
		bayes_rejection_sampling_example
		bayes_resampling_example
		)
	SET(CMAKE_EXAMPLE_DEPS mrpt-bayes mrpt-obs mrpt-gui)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: maps ===
	SET(LIST_EXAMPLES_IN_THIS_DIR
		# -----
		maps_gridmap_likelihood_characterization
		maps_gridmap_voronoi_example
	)
	SET(CMAKE_EXAMPLE_DEPS mrpt-maps mrpt-gui)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: slam ===
	SET(LIST_EXAMPLES_IN_THIS_DIR
		maps_gridmap_benchmark
		slam_icp_simple_example
		slam_icp3d_simple_example
		slam_range_only_localization_rej_sampling_example
	)
	SET(CMAKE_EXAMPLE_DEPS mrpt-slam mrpt-gui)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: topography ===
	SET(LIST_EXAMPLES_IN_THIS_DIR
		topography_gps_coords_example
		topography_coordinate_conversion_example
	)
	SET(CMAKE_EXAMPLE_DEPS mrpt-topography)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: vision ===
	SET(LIST_EXAMPLES_IN_THIS_DIR
		vision_feature_extraction
		vision_create_video_file_example
		vision_checkerboard_detectors
		vision_multiple_checkerboards
		vision_keypoint_matching_example
		vision_bundle_adj_example
		vision_stereo_calib_example
		)
	SET(CMAKE_EXAMPLE_DEPS mrpt-vision mrpt-gui)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: obs ===
	SET(LIST_EXAMPLES_IN_THIS_DIR
		obs_mox_model_rawlog
		# special utilities:
		rgbd_dataset2rawlog
		kitti_dataset2rawlog
		)
	SET(CMAKE_EXAMPLE_DEPS mrpt-obs)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: maps, gui ===
	SET(LIST_EXAMPLES_IN_THIS_DIR
		maps_laser_projection_in_images_example
		maps_observer_pattern_example
		maps_octomap_simple
		maps_gmrf_map_example
		maps_ransac_data_association
		)
	SET(CMAKE_EXAMPLE_DEPS mrpt-maps mrpt-gui)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: graphs & gui ===
	SET(LIST_EXAMPLES_IN_THIS_DIR
		graphs_astar_example
		graphs_dijkstra_example
		)
	SET(CMAKE_EXAMPLE_DEPS mrpt-graphs mrpt-gui)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: graphslam & gui ===
	SET(LIST_EXAMPLES_IN_THIS_DIR
		graphslam_example
		)
	SET(CMAKE_EXAMPLE_DEPS mrpt-graphslam mrpt-gui)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: gui & hwdrivers ===
	SET(LIST_EXAMPLES_IN_THIS_DIR
		hwdrivers_camera_capture_dialog
		hwdrivers_capture_video_opencv
		hwdrivers_capture_video_dc1394
		hwdrivers_capture_video_ffmpeg
		hwdrivers_capture_video_flycapture2
		hwdrivers_capture_video_flycapture2_stereo
		hwdrivers_enumerate_cameras1394
		hwdrivers_gps_example
		hwdrivers_joystick_example
		hwdrivers_ntrip_client_example
		hwdrivers_phidget_proximity_example
		)
	SET(CMAKE_EXAMPLE_DEPS mrpt-hwdrivers mrpt-gui)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: maps & hwdrivers ===
	SET(LIST_EXAMPLES_IN_THIS_DIR
		hwdrivers_swissranger_example
		hwdrivers_sick_serial_example
		hwdrivers_sick_eth_example
		hwdrivers_hokuyo_example
		hwdrivers_robopeaklidar_example
		hwdrivers_kinect_to_2d_scan_example
		)
	SET(CMAKE_EXAMPLE_DEPS mrpt-hwdrivers mrpt-maps)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: slam & hwdrivers ===
	SET(LIST_EXAMPLES_IN_THIS_DIR
		hwdrivers_kinect_online_offline_example
	)
	SET(CMAKE_EXAMPLE_DEPS mrpt-slam mrpt-hwdrivers)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === HWDRIVERS & VISION ===
	SET(LIST_EXAMPLES_IN_THIS_DIR
		vision_capture_video_build_pyr
		vision_stereo_rectify
		)
	SET(CMAKE_EXAMPLE_DEPS mrpt-vision mrpt-hwdrivers)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === HWDRIVERS & DETECTORS ===
	SET(LIST_EXAMPLES_IN_THIS_DIR
		detectors_face
		)
	SET(CMAKE_EXAMPLE_DEPS mrpt-slam mrpt-hwdrivers mrpt-detectors)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === OPENNI2 examples ===
	IF (MRPT_HAS_OPENNI2)
		SET(LIST_EXAMPLES_IN_THIS_DIR
			hwdrivers_openni2_driver_demo
			hwdrivers_openni2_2d_icp_slam
			)
		SET(CMAKE_EXAMPLE_DEPS mrpt-hwdrivers mrpt-gui mrpt-opengl mrpt-maps)
		GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()
	ENDIF(MRPT_HAS_OPENNI2)

	# === Navigation examples ===
	IF(BUILD_mrpt-nav)
		SET(LIST_EXAMPLES_IN_THIS_DIR
			nav_circ_robot_path_planning
			nav_rrt_planning_example
			)
		SET(CMAKE_EXAMPLE_DEPS mrpt-nav mrpt-gui)
		GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()
	ENDIF()

	# Generate the CMakeLists.txt in the "/samples" directory
	SET(CMAKE_COMMANDS_INCLUDE_EXAMPLE_DIRS ${CMAKE_COMMANDS_INCLUDE_EXAMPLE_DIRS_ROOT})
	CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/samples/CMakeLists_list_template.txt.in "${CMAKE_SOURCE_DIR}/samples/CMakeLists.txt" )
	add_subdirectory(samples)
ENDIF()
