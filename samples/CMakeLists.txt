project(ALL_EXAMPLES)

add_custom_target(examples ALL)

set(BUILD_EXAMPLES OFF CACHE BOOL "Build examples?")
if(BUILD_EXAMPLES)
	set(CMAKE_MRPT_EXAMPLES_BASE_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})

	# Generate examples_config.h
	configure_file(
    "${MRPT_SOURCE_DIR}/parse-files/examples_config.h.in"
    "${MRPT_CONFIG_FILE_INCLUDE_DIR}/mrpt/examples_config.h")

	# Generate CMakeLists.txt for each example:
	message(STATUS "Generating CMakefiles.txt for examples...")

	# ---------------------------------------------------------------
	#  MACRO for samples directories
	# ---------------------------------------------------------------
	macro(GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY)
		#TODO: Disable build if BUILD_mrpt_xxx is OFF

		# Convert CMAKE_EXAMPLE_DEPS -> CMAKE_EXAMPLE_DEPS_STRIP
		#          "mrpt-xxx mrpt-yyy" -> "xxx yyy"
		set(CMAKE_EXAMPLE_DEPS_STRIP "")
		foreach(DEP ${CMAKE_EXAMPLE_DEPS})
			# Only for "mrpt-XXX" libs:
			string(REGEX REPLACE "mrpt-(.*)" "\\1" STRIP_DEP ${DEP})
			if(NOT "${STRIP_DEP}" STREQUAL "")
				list(APPEND CMAKE_EXAMPLE_DEPS_STRIP ${STRIP_DEP})
			endif(NOT "${STRIP_DEP}" STREQUAL "")
		endforeach(DEP)

		foreach(CMAKE_MRPT_EXAMPLE_NAME ${LIST_EXAMPLES_IN_THIS_DIR})
			# Generate project file:
			configure_file(
			  ${MRPT_SOURCE_DIR}/samples/CMakeLists_template.txt.in
			  ${MRPT_SOURCE_DIR}/samples/${CMAKE_MRPT_EXAMPLE_NAME}/CMakeLists.txt
			  @ONLY)
			# And parse it:
			add_subdirectory(${CMAKE_MRPT_EXAMPLE_NAME})
		endforeach()
	endmacro()
	# ---------------------------------------------------------------
	#  END OF MACRO for samples directories
	# ---------------------------------------------------------------


	# === Depending on: core ===
	set(LIST_EXAMPLES_IN_THIS_DIR
		core_exceptions_example
		)
	set(CMAKE_EXAMPLE_DEPS mrpt-core)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: typemeta ===
	set(LIST_EXAMPLES_IN_THIS_DIR
		typemeta_TTypeName
		typemeta_StaticString
		typemeta_TEnumType
		)
	set(CMAKE_EXAMPLE_DEPS mrpt-typemeta)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: rtti ===
	set(LIST_EXAMPLES_IN_THIS_DIR
		rtti_example1
		)
	set(CMAKE_EXAMPLE_DEPS mrpt-rtti)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: serialization, io ===
	set(LIST_EXAMPLES_IN_THIS_DIR
		# ----
		serialization_json_example
		serialization_stl
		serialization_variant_example
		# ----
		io_pipes_example
		io_compress_example
		)
	set(CMAKE_EXAMPLE_DEPS mrpt-serialization mrpt-io mrpt-poses)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: db ===
	set(LIST_EXAMPLES_IN_THIS_DIR
		db_example
		)
	set(CMAKE_EXAMPLE_DEPS mrpt-db mrpt-io mrpt-serialization)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: system ===
	set(LIST_EXAMPLES_IN_THIS_DIR
		system_datetime_example
		system_directory_explorer_example
		system_file_system_watcher
		system_backtrace_example
		system_dirs_files_manipulation
		system_params_by_name
		)
	set(CMAKE_EXAMPLE_DEPS mrpt-system mrpt-poses mrpt-io)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: mrpt-math mrpt-random mrpt-gui ===
	set(LIST_EXAMPLES_IN_THIS_DIR
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
	set(CMAKE_EXAMPLE_DEPS mrpt-math mrpt-random mrpt-gui)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	set(LIST_EXAMPLES_IN_THIS_DIR
		math_polyhedron_intersection_example
		)
	set(CMAKE_EXAMPLE_DEPS mrpt-math mrpt-random mrpt-gui mrpt-maps)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: mrpt-poses ===
	set(LIST_EXAMPLES_IN_THIS_DIR
		poses_geometry_3D_example
		poses_pdfs_example
		poses_se3_lie_example
		poses_quaternions_example
		poses_sog_merge_example
		poses_unscented_transform_example
		)
	set(CMAKE_EXAMPLE_DEPS mrpt-poses mrpt-io mrpt-gui)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: mrpt-comms ===
	set(LIST_EXAMPLES_IN_THIS_DIR
		comms_http_client
		comms_nodelets_example
		comms_serial_port_example
		comms_socket_example
		comms_ftdi_usb_enumerate_example
		)
	set(CMAKE_EXAMPLE_DEPS mrpt-comms mrpt-serialization mrpt-poses)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: mrpt-gui, mrpt-img, mrpt-opengl ===
	set(LIST_EXAMPLES_IN_THIS_DIR
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
	set(CMAKE_EXAMPLE_DEPS mrpt-gui)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: mrpt-gui, mrpt-hwdrivers ===
	set(LIST_EXAMPLES_IN_THIS_DIR
		opengl_video_demo
		opengl_video_viewport_demo
		)
	set(CMAKE_EXAMPLE_DEPS mrpt-gui mrpt-hwdrivers)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: mrpt-gui, mrpt-maps ===
	set(LIST_EXAMPLES_IN_THIS_DIR
		opengl_ray_trace_example
		)
	set(CMAKE_EXAMPLE_DEPS mrpt-gui mrpt-maps)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: mrpt-random ===
	set(LIST_EXAMPLES_IN_THIS_DIR
		random_examples
		)
	set(CMAKE_EXAMPLE_DEPS mrpt-random mrpt-gui)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: mrpt-bayes, mrpt-obs, mrpt-gui ===
	set(LIST_EXAMPLES_IN_THIS_DIR
		bayes_tracking_example
		bayes_rejection_sampling_example
		bayes_resampling_example
		)
	set(CMAKE_EXAMPLE_DEPS mrpt-bayes mrpt-obs mrpt-gui)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: maps ===
	set(LIST_EXAMPLES_IN_THIS_DIR
		# -----
		maps_gridmap_likelihood_characterization
		maps_gridmap_voronoi_example
	)
	set(CMAKE_EXAMPLE_DEPS mrpt-maps mrpt-gui)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: slam ===
	set(LIST_EXAMPLES_IN_THIS_DIR
		maps_gridmap_benchmark
		slam_icp_simple_example
		slam_icp3d_simple_example
		slam_range_only_localization_rej_sampling_example
	)
	set(CMAKE_EXAMPLE_DEPS mrpt-slam mrpt-gui)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: topography ===
	set(LIST_EXAMPLES_IN_THIS_DIR
		topography_gps_coords_example
		topography_coordinate_conversion_example
	)
	set(CMAKE_EXAMPLE_DEPS mrpt-topography)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: vision ===
	set(LIST_EXAMPLES_IN_THIS_DIR
		vision_create_video_file_example
		vision_checkerboard_detectors
		vision_feature_extraction
		vision_keypoint_matching_example
		vision_multiple_checkerboards
		vision_stereo_calib_example
		)
	set(CMAKE_EXAMPLE_DEPS mrpt-vision mrpt-gui)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	if(MRPT_ALLOW_LGPLV3)
		# === Depending on: vision-lgpl ===
		set(LIST_EXAMPLES_IN_THIS_DIR
			vision_bundle_adj_example
			)
		set(CMAKE_EXAMPLE_DEPS mrpt-vision-lgpl mrpt-gui)
		GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()
	endif()

	# === Depending on: obs ===
	set(LIST_EXAMPLES_IN_THIS_DIR
		obs_mox_model_rawlog
		# special utilities:
		rgbd_dataset2rawlog
		kitti_dataset2rawlog
		)
	set(CMAKE_EXAMPLE_DEPS mrpt-obs)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: maps, gui ===
	set(LIST_EXAMPLES_IN_THIS_DIR
		maps_laser_projection_in_images_example
		maps_observer_pattern_example
		maps_octomap_simple
		maps_gmrf_map_example
		maps_ransac_data_association
		)
	set(CMAKE_EXAMPLE_DEPS mrpt-maps mrpt-gui)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: graphs & gui ===
	set(LIST_EXAMPLES_IN_THIS_DIR
		graphs_astar_example
		graphs_dijkstra_example
		)
	set(CMAKE_EXAMPLE_DEPS mrpt-graphs mrpt-gui)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: graphslam & gui ===
	set(LIST_EXAMPLES_IN_THIS_DIR
		graphslam_example
		)
	set(CMAKE_EXAMPLE_DEPS mrpt-graphslam mrpt-gui)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: gui & hwdrivers ===
	set(LIST_EXAMPLES_IN_THIS_DIR
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
	set(CMAKE_EXAMPLE_DEPS mrpt-hwdrivers mrpt-gui)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: maps & hwdrivers ===
	set(LIST_EXAMPLES_IN_THIS_DIR
		hwdrivers_swissranger_example
		hwdrivers_sick_serial_example
		hwdrivers_sick_eth_example
		hwdrivers_hokuyo_example
		hwdrivers_robopeaklidar_example
		hwdrivers_kinect_to_2d_scan_example
		)
	set(CMAKE_EXAMPLE_DEPS mrpt-hwdrivers mrpt-maps)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: slam & hwdrivers ===
	set(LIST_EXAMPLES_IN_THIS_DIR
		hwdrivers_kinect_online_offline_example
	)
	set(CMAKE_EXAMPLE_DEPS mrpt-slam mrpt-hwdrivers)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === HWDRIVERS & VISION ===
	set(LIST_EXAMPLES_IN_THIS_DIR
		vision_capture_video_build_pyr
		vision_stereo_rectify
		)
	set(CMAKE_EXAMPLE_DEPS mrpt-vision mrpt-hwdrivers)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === HWDRIVERS & DETECTORS ===
	set(LIST_EXAMPLES_IN_THIS_DIR
		detectors_face
		)
	set(CMAKE_EXAMPLE_DEPS mrpt-slam mrpt-hwdrivers mrpt-detectors)
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === OPENNI2 examples ===
	if (MRPT_HAS_OPENNI2)
		set(LIST_EXAMPLES_IN_THIS_DIR
			hwdrivers_openni2_driver_demo
			hwdrivers_openni2_2d_icp_slam
			)
		set(CMAKE_EXAMPLE_DEPS mrpt-hwdrivers mrpt-gui mrpt-opengl mrpt-maps)
		GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()
	endif(MRPT_HAS_OPENNI2)

	# === Navigation examples ===
	if(BUILD_mrpt-nav)
		set(LIST_EXAMPLES_IN_THIS_DIR
			nav_circ_robot_path_planning
			nav_rrt_planning_example
			)
		set(CMAKE_EXAMPLE_DEPS mrpt-nav mrpt-gui)
		GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()
	endif()

	message(STATUS "Done.")
endif()
