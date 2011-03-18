# This file is included from the main CMakeLists.txt
#
SET( BUILD_EXAMPLES OFF CACHE BOOL "Build examples?")
IF(BUILD_EXAMPLES)
	SET(CMAKE_MRPT_EXAMPLES_BASE_DIRECTORY "${CMAKE_SOURCE_DIR}/samples/")
	# Fix "\" --> "\\" for windows:
	string(REPLACE "\\" "\\\\" CMAKE_MRPT_EXAMPLES_BASE_DIRECTORY ${CMAKE_MRPT_EXAMPLES_BASE_DIRECTORY})

	MESSAGE(STATUS "Parsing 'examples_config.h.in'")
	CONFIGURE_FILE("${CMAKE_SOURCE_DIR}/parse-files/examples_config.h.in" "${MRPT_CONFIG_FILE_INCLUDE_DIR}/mrpt/examples_config.h")

	# Generate CMakeLists.txt from the template project file for examples:
	MESSAGE(STATUS "Generating CMakefiles.txt for examples...")

	# ---------------------------------------------------------------
	#  MACRO for samples directories
	# ---------------------------------------------------------------
	MACRO(GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY)
		FOREACH(CMAKE_MRPT_EXAMPLE_NAME ${LIST_EXAMPLES_IN_THIS_DIR})
			#MESSAGE(STATUS "Example: ${CMAKE_MRPT_EXAMPLE_NAME}")
			# Generate project file:
			CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/samples/CMakeLists_template.txt.in ${CMAKE_SOURCE_DIR}/samples/${CMAKE_MRPT_EXAMPLE_NAME}/CMakeLists.txt @ONLY)
			SET(CMAKE_COMMANDS_INCLUDE_EXAMPLE_DIRS_ROOT "${CMAKE_COMMANDS_INCLUDE_EXAMPLE_DIRS_ROOT}\nadd_subdirectory(${CMAKE_MRPT_EXAMPLE_NAME})")
		ENDFOREACH(CMAKE_MRPT_EXAMPLE_NAME)
	ENDMACRO(GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY)
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
		exceptionDemo
		http_tests
		critSectionDeadLock
		threadsTest
		fileSystemWatcher
		geometry3D
		poses
		se3
		type_name
		)
	SET(CMAKE_EXAMPLE_DEPS mrpt-base)
	SET(CMAKE_EXAMPLE_LINK_LIBS ${MRPT_LINKER_LIBS})
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: mrpt-base, mrpt-gui ===
	#  list of examples for each directory:
	SET(LIST_EXAMPLES_IN_THIS_DIR
		display3D
		opengl_objects_demo
		fbo_render_test
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
		pathPlanning
		gridRawlogSimulator
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
		dijkstra-example
		setOfTexturedTrianglesTest
		gui_windows_events
		a_starAlgorithm
		quaternions
		unscented_transform_test
		grab3Dvideo
		coordinate_conversions
		params-by-name		
		checkerboardDetectors
		multipleCheckerboards		
		voronoi_test
		graph_slam_demo
		keypoint_matching
		)
	SET(CMAKE_EXAMPLE_DEPS mrpt-slam mrpt-vision mrpt-gui mrpt-topography)
	SET(CMAKE_EXAMPLE_LINK_LIBS ${MRPT_LINKER_LIBS})
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# === Depending on: vision & gui ===
	#  list of examples for each directory:
	SET(LIST_EXAMPLES_IN_THIS_DIR
		bundle_adj_full_demo
		)
	SET(CMAKE_EXAMPLE_DEPS mrpt-vision mrpt-gui)
	SET(CMAKE_EXAMPLE_LINK_LIBS ${MRPT_LINKER_LIBS})
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()
	
	# === Depending on: gui & hwdrivers ===
	#  list of examples for each directory:
	SET(LIST_EXAMPLES_IN_THIS_DIR
		opengl_video_demo
		opengl_video_viewport_demo
		)
	SET(CMAKE_EXAMPLE_DEPS mrpt-hwdrivers mrpt-gui)
	SET(CMAKE_EXAMPLE_LINK_LIBS ${MRPT_LINKER_LIBS})
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()
	

	# === HWDRIVERS ===
	IF(BUILD_HWDRIVERS)
		#  list of examples for each directory:
		SET(LIST_EXAMPLES_IN_THIS_DIR
			swissranger_cam_demo
			GPS_test
			sonar_SRF10_test
			eNoses_test
			SerialPort_test
			FTDI_USB_enumerate_test
			SICK_laser_test
			HOKUYO_laser_test
			joystick
			pioneerRobotDemo
			ptuDPerception
			ptuHokuyo
			tuMicos
			captureVideoFFmpeg
			SICK_laser_serial_test
			ntrip-client
			SICK_lms100eth_test
			eNeck_test
			# rovio_test
			#tracking_planes_test
			cameraCaptureAskDialog
			eNosesRealtime_test
			IRBoard_test
			captureVideoOpenCV
			captureVideoDC1394
			enumerateCameras1394
			phidgetProximitySensor
			)
		SET(CMAKE_EXAMPLE_DEPS mrpt-slam mrpt-hwdrivers)
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

	# Generate the CMakeLists.txt in the "/samples" directory
	SET(CMAKE_COMMANDS_INCLUDE_EXAMPLE_DIRS ${CMAKE_COMMANDS_INCLUDE_EXAMPLE_DIRS_ROOT})
	CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/samples/CMakeLists_list_template.txt.in "${CMAKE_SOURCE_DIR}/samples/CMakeLists.txt" )
	add_subdirectory(samples)
ENDIF(BUILD_EXAMPLES)
