# Usage: cmake -P generate_cmake_files.cmake

# ---------------------------------------------------------------
#  MACRO for samples directories
# ---------------------------------------------------------------
macro(GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY)
  # Pre-calculate unrolled strings for the template
  set(TPL_FIND_PACKAGES "")
  set(TPL_LINK_LIBRARIES "")

  foreach(DEP ${CMAKE_EXAMPLE_DEPS})
    # v3 logic: mrpt::mrpt_core -> find_package(mrpt_core) + link mrpt::mrpt_core
    if (DEP MATCHES "^mrpt::(mrpt_.*)")
      set(RAW_MOD_NAME "${CMAKE_MATCH_1}")
      string(APPEND TPL_FIND_PACKAGES "find_package(${RAW_MOD_NAME} REQUIRED)\n")
      string(APPEND TPL_LINK_LIBRARIES "${DEP} ")
    endif()
  endforeach()

  foreach(CMAKE_MRPT_EXAMPLE_NAME ${LIST_EXAMPLES_IN_THIS_DIR})
    message(STATUS "Configuring example: ${CMAKE_MRPT_EXAMPLE_NAME}")
    
    # Generate the file
    configure_file(
      "${CMAKE_CURRENT_SOURCE_DIR}/CMakeLists_template.txt.in"
      "${CMAKE_CURRENT_SOURCE_DIR}/${CMAKE_MRPT_EXAMPLE_NAME}/CMakeLists.txt"
      @ONLY)

    # Only add_subdirectory if we are in project mode (not script mode -P)
    if(NOT CMAKE_SCRIPT_MODE_FILE)
       add_subdirectory("${CMAKE_MRPT_EXAMPLE_NAME}")
    endif()
  endforeach()
endmacro()


# === Depending on: core ===
set(LIST_EXAMPLES_IN_THIS_DIR
  core_exceptions_example
  core_backtrace_example
  )
set(CMAKE_EXAMPLE_DEPS mrpt::mrpt_core)
GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

# === Depending on: typemeta ===
set(LIST_EXAMPLES_IN_THIS_DIR
  typemeta_TTypeName
  typemeta_StaticString
  typemeta_TEnumType
  )
set(CMAKE_EXAMPLE_DEPS mrpt::mrpt_typemeta)
GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

# === Depending on: rtti ===
set(LIST_EXAMPLES_IN_THIS_DIR
  rtti_example1
  )
set(CMAKE_EXAMPLE_DEPS mrpt::mrpt_rtti)
GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

# === Depending on: containers, systems ===
set(LIST_EXAMPLES_IN_THIS_DIR
  containers_yaml_example
  containers_params_by_name
  )
set(CMAKE_EXAMPLE_DEPS mrpt::mrpt_containers mrpt::mrpt_system)
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
set(CMAKE_EXAMPLE_DEPS mrpt::mrpt_serialization mrpt::mrpt_io mrpt::mrpt_poses)
GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

# === Depending on: system ===
set(LIST_EXAMPLES_IN_THIS_DIR
  system_datetime_example
  system_directory_explorer_example
  system_file_system_watcher
  system_dirs_files_manipulation
  system_progress_bar
  )
set(CMAKE_EXAMPLE_DEPS mrpt::mrpt_system mrpt::mrpt_poses mrpt::mrpt_io)
GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

# === Depending on: system+math+gui ===
set(LIST_EXAMPLES_IN_THIS_DIR
  system_control_rate_timer_example
  # -------
  # Plus: a couple of other opengl_* samples listed below since they need mrpt::mrpt_maps also
  opengl_offscreen_render_example
  opengl_multithread_rendering
  opengl_octree_render_huge_pointcloud
  opengl_texture_sizes_test
  opengl_skybox_example
  )
set(CMAKE_EXAMPLE_DEPS mrpt::mrpt_system mrpt::mrpt_math mrpt::mrpt_gui)
GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

# === Depending on: mrpt::mrpt_math mrpt::mrpt_random mrpt::mrpt_gui ===
set(LIST_EXAMPLES_IN_THIS_DIR
  math_csparse_example
  math_matrix_example
  math_optimize_lm_example
  math_slerp_example
  math_spline_interpolation
  math_leastsquares_example
  math_ransac_plane3d_example
  math_ransac_plane3d_example2
  math_ransac_examples
  math_model_search_example
  math_polygon_split
  math_polygon_intersection
  )
set(CMAKE_EXAMPLE_DEPS mrpt::mrpt_math mrpt::mrpt_random mrpt::mrpt_gui mrpt::mrpt_maps)
GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

set(LIST_EXAMPLES_IN_THIS_DIR
  math_polyhedron_intersection_example
  )
set(CMAKE_EXAMPLE_DEPS mrpt::mrpt_math mrpt::mrpt_random mrpt::mrpt_gui mrpt::mrpt_maps)
GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

# === Depending on: mrpt::mrpt_poses ===
set(LIST_EXAMPLES_IN_THIS_DIR
  poses_geometry_3D_example
  poses_pdfs_example
  poses_se3_lie_example
  poses_quaternions_example
  poses_sog_merge_example
  poses_unscented_transform_example
  )
set(CMAKE_EXAMPLE_DEPS mrpt::mrpt_poses mrpt::mrpt_io mrpt::mrpt_gui)
GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

# === Depending on: mrpt::mrpt_comms ===
set(LIST_EXAMPLES_IN_THIS_DIR
  comms_http_client
  comms_nodelets_example
  comms_serial_port_example
  comms_socket_example
  comms_ftdi_usb_enumerate_example
  )
set(CMAKE_EXAMPLE_DEPS mrpt::mrpt_comms mrpt::mrpt_serialization mrpt::mrpt_poses)
GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

# === Depending on: mrpt::mrpt_gui, mrpt::mrpt_img, mrpt::mrpt_opengl, mrpt::mrpt_maps ===
set(LIST_EXAMPLES_IN_THIS_DIR
  # -------
  # opengl demos that also requires mrpt::mrpt_maps
  opengl_objects_demo
  opengl_custom_shaders_demo
  # -------
  img_basic_example
  img_convolution_fft
  img_fft_example
  img_gauss_filtering_example
  # -------
  gui_capture_render_to_img_example
  gui_display3D_example
  gui_display3D_custom_render
  gui_fbo_render_example
  gui_display_plots
  gui_nanogui_demo
  gui_text_fonts_example
  gui_gravity3d_example
  gui_windows_events
  gui_depth_camera_distortion
  )
set(CMAKE_EXAMPLE_DEPS mrpt::mrpt_gui mrpt::mrpt_maps)
GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

# === Depending on: mrpt::mrpt_gui, mrpt::mrpt_hwdrivers ===
set(LIST_EXAMPLES_IN_THIS_DIR
  opengl_video_demo
  opengl_video_viewport_demo
  )
set(CMAKE_EXAMPLE_DEPS mrpt::mrpt_gui mrpt::mrpt_hwdrivers)
GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

# === Depending on: mrpt::mrpt_gui, mrpt::mrpt_maps ===
set(LIST_EXAMPLES_IN_THIS_DIR
  opengl_ray_trace_example
  )
set(CMAKE_EXAMPLE_DEPS mrpt::mrpt_gui mrpt::mrpt_maps)
GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

# === Depending on: mrpt::mrpt_random ===
set(LIST_EXAMPLES_IN_THIS_DIR
  random_examples
  )
set(CMAKE_EXAMPLE_DEPS mrpt::mrpt_random mrpt::mrpt_gui)
GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

# === Depending on: mrpt::mrpt_bayes, mrpt::mrpt_obs, mrpt::mrpt_gui ===
set(LIST_EXAMPLES_IN_THIS_DIR
  bayes_tracking_example
  bayes_rejection_sampling_example
  bayes_resampling_example
  )
set(CMAKE_EXAMPLE_DEPS mrpt::mrpt_bayes mrpt::mrpt_obs mrpt::mrpt_gui)
GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

# === Depending on: maps ===
set(LIST_EXAMPLES_IN_THIS_DIR
  # -----
  maps_gridmap_likelihood_characterization
  maps_gridmap_voronoi_example
  obs_motion_model_demo
)
  set(CMAKE_EXAMPLE_DEPS mrpt::mrpt_maps mrpt::mrpt_gui)
GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

# === Depending on: slam ===
set(LIST_EXAMPLES_IN_THIS_DIR
  maps_gridmap_benchmark
  slam_icp_simple_example
  slam_icp3d_simple_example
  slam_range_only_localization_rej_sampling_example
)
  set(CMAKE_EXAMPLE_DEPS mrpt::mrpt_slam mrpt::mrpt_gui)
GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

# === Depending on: topography ===
set(LIST_EXAMPLES_IN_THIS_DIR
  topography_gps_coords_example
  topography_coordinate_conversion_example
)
  set(CMAKE_EXAMPLE_DEPS mrpt::mrpt_topography)
GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

# === Depending on: vision ===
set(LIST_EXAMPLES_IN_THIS_DIR
  vision_create_video_file_example
  vision_checkerboard_detectors
  vision_feature_extraction
  vision_keypoint_matching_example
  vision_stereo_calib_example
  )
set(CMAKE_EXAMPLE_DEPS mrpt::mrpt_vision mrpt::mrpt_gui)
GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()


# === Depending on: obs ===
set(LIST_EXAMPLES_IN_THIS_DIR
  obs_mox_model_rawlog
  # special utilities:
  rgbd_dataset2rawlog
  kitti_dataset2rawlog
  )
set(CMAKE_EXAMPLE_DEPS mrpt::mrpt_obs)
GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

# === Depending on: maps, gui ===
set(LIST_EXAMPLES_IN_THIS_DIR
  maps_laser_projection_in_images_example
  maps_gridmap3D_simple
  maps_observer_pattern_example
  maps_octomap_simple
  maps_voxelmap_simple
  maps_voxelmap_from_tum_dataset
  maps_gmrf_map_example
  maps_ransac_data_association
  )
set(CMAKE_EXAMPLE_DEPS mrpt::mrpt_maps mrpt::mrpt_gui)
GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

# === Depending on: graphs & gui ===
set(LIST_EXAMPLES_IN_THIS_DIR
  graphs_astar_example
  graphs_dijkstra_example
  )
set(CMAKE_EXAMPLE_DEPS mrpt::mrpt_graphs mrpt::mrpt_gui)
GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

# === Depending on: graphslam & gui ===
set(LIST_EXAMPLES_IN_THIS_DIR
  graphslam_example
  )
set(CMAKE_EXAMPLE_DEPS mrpt::mrpt_graphslam mrpt::gui)
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
set(CMAKE_EXAMPLE_DEPS mrpt::mrpt_hwdrivers mrpt::mrpt_gui)
GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

# === Depending on: maps & hwdrivers ===
set(LIST_EXAMPLES_IN_THIS_DIR
  hwdrivers_sick_serial_example
  hwdrivers_sick_eth_example
  hwdrivers_hokuyo_example
  hwdrivers_robopeaklidar_example
  hwdrivers_kinect_to_2d_scan_example
  hwdrivers_taobotics_imu
  )
set(CMAKE_EXAMPLE_DEPS mrpt::mrpt_hwdrivers mrpt::mrpt_maps)
GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

# === Depending on: slam & hwdrivers ===
set(LIST_EXAMPLES_IN_THIS_DIR
  hwdrivers_kinect_online_offline_example
  hwdrivers_mynteye_icp
)
  set(CMAKE_EXAMPLE_DEPS mrpt::mrpt_slam mrpt::mrpt_hwdrivers)
GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

# === HWDRIVERS & VISION ===
set(LIST_EXAMPLES_IN_THIS_DIR
  vision_capture_video_build_pyr
  vision_stereo_rectify
  )
set(CMAKE_EXAMPLE_DEPS mrpt::mrpt_vision mrpt::mrpt_hwdrivers)
GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

# === OPENNI2 examples ===
set(LIST_EXAMPLES_IN_THIS_DIR
  hwdrivers_openni2_driver_demo
  hwdrivers_openni2_2d_icp_slam
  )
set(CMAKE_EXAMPLE_DEPS mrpt::mrpt_hwdrivers mrpt::mrpt_gui mrpt::mrpt_opengl mrpt::mrpt_maps)
GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

# === Navigation examples ===
set(LIST_EXAMPLES_IN_THIS_DIR
  nav_circ_robot_path_planning
  nav_rrt_planning_example
  )
set(CMAKE_EXAMPLE_DEPS mrpt::mrpt_nav mrpt::mrpt_gui)
GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

message(STATUS "Done.")
