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
	ENDMACRO(ADD_SAMPLES_DIRECTORY)
	# ---------------------------------------------------------------
	#  END OF MACRO for samples directories
	# ---------------------------------------------------------------

	# -----------------------------------------------------------------
	# This loop is generic, do not modify it...
	#  modify the above variable and/or the list_examples.txt files!
	# -----------------------------------------------------------------
	SET(CMAKE_COMMANDS_INCLUDE_EXAMPLE_DIRS_ROOT "")

	# === Depending on: mrpt-base ===
	#  list of examples for each directory:
	SET(LIST_EXAMPLES_IN_THIS_DIR
		db
		times
		SocketsTest
		directoryExplorer
		http_tests
		critSectionDeadLock
		threadsTest
		fileSystemWatcher
		geometry3D
		poses
		se3
		csparse_demo
		threadsPipe
		dirs_files_manipulation
		backtrace-example
		)
	SET(CMAKE_EXAMPLE_DEPS mrpt-base)
	SET(CMAKE_EXAMPLE_LINK_LIBS ${MRPT_LINKER_LIBS})
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: mrpt-base, mrpt-gui ===
	#  list of examples for each directory:
	SET(LIST_EXAMPLES_IN_THIS_DIR
		display3D
		display3D_custom_render
		opengl_objects_demo
		fbo_render_test
		offscreen-render
		octree_render_huge_pointcloud
		imageBasics
		imageCorrelation
		random
		imageConvolutionFFT
		imageFFT
		matrix
		displayPlots
		textFonts
		optimize-lm
		kmeans
		slerp_demo
		texture_sizes_test
		)
	SET(CMAKE_EXAMPLE_DEPS mrpt-base mrpt-gui)
	SET(CMAKE_EXAMPLE_LINK_LIBS ${MRPT_LINKER_LIBS})
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: mrpt-base, mrpt-bayes, mrpt-obs, mrpt-gui ===
	#  list of examples for each directory:
	SET(LIST_EXAMPLES_IN_THIS_DIR
		bayesianTracking
		)
	SET(CMAKE_EXAMPLE_DEPS mrpt-base mrpt-bayes mrpt-obs mrpt-gui)
	SET(CMAKE_EXAMPLE_LINK_LIBS ${MRPT_LINKER_LIBS})
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()


	# === Depending on: base, obs, maps, etc... ===
	#  list of examples for each directory:
	SET(LIST_EXAMPLES_IN_THIS_DIR
		icp
		rejectionSampling
		stringList
		RangeOnlyLocalization_RejectionSampling
		benchmark-gridmaps
		gridMapLikelihoodCharacterization
		gauss_img_filtering
		test-compress
		spline_interpolation
		gravity3d
		resampling-test
		feature_extraction
		gps-coordinates
		sog-merge
		laserProjectionInImages
		leastSquares
		rayTrace
		icp3D
		ransac-demo-plane3D
		ransac-demo-applications
		model_search_test
		createVideoFile
		polyhedronIntersection
		observer_pattern
		smart_pointers_test
		stl_containers_serialize
		polygonSplit
		setOfTexturedTrianglesTest
		gui_windows_events
		quaternions
		unscented_transform_test
		grab3Dvideo
		coordinate_conversions
		params-by-name
		checkerboardDetectors
		multipleCheckerboards
		voronoi_test
		keypoint_matching
		)
	SET(CMAKE_EXAMPLE_DEPS mrpt-slam mrpt-vision mrpt-gui mrpt-topography)
	SET(CMAKE_EXAMPLE_LINK_LIBS ${MRPT_LINKER_LIBS})
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: nav ===
	#  list of examples for each directory:
	SET(LIST_EXAMPLES_IN_THIS_DIR
		pathPlanning
		)
	SET(CMAKE_EXAMPLE_DEPS mrpt-nav mrpt-gui)
	SET(CMAKE_EXAMPLE_LINK_LIBS ${MRPT_LINKER_LIBS})
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: obs ===
	#  list of examples for each directory:
	SET(LIST_EXAMPLES_IN_THIS_DIR
	rgbd_dataset2rawlog
	kitti_dataset2rawlog
	)
	SET(CMAKE_EXAMPLE_DEPS mrpt-obs)
	SET(CMAKE_EXAMPLE_LINK_LIBS ${MRPT_LINKER_LIBS})
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: maps, gui ===
	#  list of examples for each directory:
	SET(LIST_EXAMPLES_IN_THIS_DIR
		octomap_simple
		gmrf_map_demo
		)
	SET(CMAKE_EXAMPLE_DEPS mrpt-maps mrpt-gui)
	SET(CMAKE_EXAMPLE_LINK_LIBS ${MRPT_LINKER_LIBS})
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: maps, tfest, gui ===
	#  list of examples for each directory:
	SET(LIST_EXAMPLES_IN_THIS_DIR
		ransac-data-association
		)
	SET(CMAKE_EXAMPLE_DEPS mrpt-maps mrpt-gui mrpt-tfest)
	SET(CMAKE_EXAMPLE_LINK_LIBS ${MRPT_LINKER_LIBS})
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: graphs & gui ===
	#  list of examples for each directory:
	SET(LIST_EXAMPLES_IN_THIS_DIR
		a_starAlgorithm
		dijkstra-example
		type_name
		)
	SET(CMAKE_EXAMPLE_DEPS mrpt-graphs mrpt-gui)
	SET(CMAKE_EXAMPLE_LINK_LIBS ${MRPT_LINKER_LIBS})
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()


	# === Depending on: graphslam & gui ===
	#  list of examples for each directory:
	SET(LIST_EXAMPLES_IN_THIS_DIR
		graph_slam_demo
		)
	SET(CMAKE_EXAMPLE_DEPS mrpt-graphslam mrpt-gui)
	SET(CMAKE_EXAMPLE_LINK_LIBS ${MRPT_LINKER_LIBS})
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()


	# === Depending on: vision & gui ===
	#  list of examples for each directory:
	SET(LIST_EXAMPLES_IN_THIS_DIR
		bundle_adj_full_demo
		stereo-calib-demo
		)
	SET(CMAKE_EXAMPLE_DEPS mrpt-vision mrpt-gui)
	SET(CMAKE_EXAMPLE_LINK_LIBS ${MRPT_LINKER_LIBS})
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === HWDRIVERS ===
	IF(BUILD_HWDRIVERS)

	        # === Depending on: gui & hwdrivers ===
	        #  list of examples for each directory:
	        SET(LIST_EXAMPLES_IN_THIS_DIR
	                opengl_video_demo
	                opengl_video_viewport_demo
			captureVideoOpenCV
			captureVideoDC1394
			captureVideoFlyCapture2
			captureVideoFlyCapture2_stereo
			enumerateCameras1394
			GPS_test
			sonar_SRF10_test
			eNoses_test
			SerialPort_test
			FTDI_USB_enumerate_test
			joystick
			captureVideoFFmpeg
			ptuDPerception
			tuMicos
			ntrip-client
			eNeck_test
			cameraCaptureAskDialog
			eNosesRealtime_test
			phidgetProximitySensor
			NIDAQ_test
			)
	        SET(CMAKE_EXAMPLE_DEPS mrpt-hwdrivers mrpt-gui)
	        SET(CMAKE_EXAMPLE_LINK_LIBS ${MRPT_LINKER_LIBS})
	        GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	        # === Depending on: maps & hwdrivers ===
	        #  list of examples for each directory:
	        SET(LIST_EXAMPLES_IN_THIS_DIR
			swissranger_cam_demo
			SICK_laser_serial_test
			SICK_laser_test
			HOKUYO_laser_test
			RoboPeakLidar_laser_test
			SICK_lms100eth_test
			kinect-to-2d-laser-demo
	                )
	        SET(CMAKE_EXAMPLE_DEPS mrpt-hwdrivers mrpt-maps)
	        SET(CMAKE_EXAMPLE_LINK_LIBS ${MRPT_LINKER_LIBS})
	        GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

		# === Depending on: slam & hwdrivers ===
		#  list of examples for each directory:
		SET(LIST_EXAMPLES_IN_THIS_DIR
			kinect_online_offline_demo
			MOXmodel-rawlog
			)
		SET(CMAKE_EXAMPLE_DEPS mrpt-slam mrpt-hwdrivers)
		SET(CMAKE_EXAMPLE_LINK_LIBS ${MRPT_LINKER_LIBS})
		GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

		# === HWDRIVERS & VISION ===
		SET(LIST_EXAMPLES_IN_THIS_DIR
			captureVideoAndBuildPyr
			stereoRectify
			)
		SET(CMAKE_EXAMPLE_DEPS mrpt-vision mrpt-hwdrivers)
		SET(CMAKE_EXAMPLE_LINK_LIBS ${MRPT_LINKER_LIBS})
		GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

		# === HWDRIVERS & DETECTORS ===
		SET(LIST_EXAMPLES_IN_THIS_DIR
			face_detection
			)
		SET(CMAKE_EXAMPLE_DEPS mrpt-slam mrpt-hwdrivers mrpt-detectors)
		SET(CMAKE_EXAMPLE_LINK_LIBS ${MRPT_LINKER_LIBS})
		GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()


	ENDIF(BUILD_HWDRIVERS)

	# === OPENNI2 examples ===
	IF (MRPT_HAS_OPENNI2)
		SET(LIST_EXAMPLES_IN_THIS_DIR
			openNI2_RGBD_demo
			openNI2_proximity_demo
			openNI2_driver_demo
			openNI2_2d-icp-slam
			openNI2_to_rawlog)

		SET(CMAKE_EXAMPLE_DEPS mrpt-base mrpt-hwdrivers mrpt-gui mrpt-opengl mrpt-maps)
		SET(CMAKE_EXAMPLE_LINK_LIBS ${MRPT_LINKER_LIBS} "\${OPENNI2_LIBRARIES}")
		GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()
	ENDIF(MRPT_HAS_OPENNI2)

	# === PbMap examples ===
	IF(BUILD_mrpt-pbmap)
		ADD_SAMPLES_DIRECTORY(pbmap-examples)
#		SET(LIST_EXAMPLES_IN_THIS_DIR
#			pbmap_example
#			pbmap_visualizer
#			)
#		SET(CMAKE_EXAMPLE_DEPS mrpt-pbmap mrpt-gui)
#		SET(CMAKE_EXAMPLE_LINK_LIBS ${MRPT_LINKER_LIBS})
#		GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()
	ENDIF(BUILD_mrpt-pbmap)

	# === Navigation examples ===
	IF(BUILD_mrpt-nav)
		SET(LIST_EXAMPLES_IN_THIS_DIR
			rrt_planning_example
			)
		SET(CMAKE_EXAMPLE_DEPS mrpt-nav mrpt-gui)
		SET(CMAKE_EXAMPLE_LINK_LIBS ${MRPT_LINKER_LIBS})
		GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()
	ENDIF(BUILD_mrpt-nav)


	# Generate the CMakeLists.txt in the "/samples" directory
	SET(CMAKE_COMMANDS_INCLUDE_EXAMPLE_DIRS ${CMAKE_COMMANDS_INCLUDE_EXAMPLE_DIRS_ROOT})
	CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/samples/CMakeLists_list_template.txt.in "${CMAKE_SOURCE_DIR}/samples/CMakeLists.txt" )
	add_subdirectory(samples)
ENDIF(BUILD_EXAMPLES)
