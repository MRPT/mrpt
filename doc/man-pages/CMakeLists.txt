include(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

# ----------------------------------------------------------------------------
# Targets for building man-pages of the MRPT
# ----------------------------------------------------------------------------

project(all_man_pages)

find_program(PROG_POD2MAN pod2man)
find_program(PROG_GZIP gzip)

if (PROG_GZIP AND PROG_POD2MAN)
	mark_as_advanced(PROG_POD2MAN)
	mark_as_advanced(PROG_GZIP)

	# Make sure the output dir exists:
	file(MAKE_DIRECTORY "${MRPT_BINARY_DIR}/man-pages")

	# Macro for declaring man-pages targets:
	macro(CREATE_MANPAGE_PROJECT appname)
		add_custom_target(man_page_${appname}
			COMMAND 			pod2man --center="Mobile Robot Programming Toolkit - MRPT"  ${MRPT_SOURCE_DIR}/doc/man-pages/pod/${appname}.pod |gzip -c > ${MRPT_BINARY_DIR}/man-pages/${appname}.1.gz
			WORKING_DIRECTORY	${MRPT_SOURCE_DIR}/
			COMMENT 		"Generating MRPT man pages"
			)
		add_dependencies(man_pages_all man_page_${appname})
		if (UNIX)
			install(FILES "${MRPT_BINARY_DIR}/man-pages/${appname}.1.gz" DESTINATION ${mrpt_apps_INSTALL_PREFIX}share/man/man1/ )
		endif()
	endmacro(CREATE_MANPAGE_PROJECT)

	# The main (empty) target:
	add_custom_target(man_pages_all ALL)

	CREATE_MANPAGE_PROJECT(rawlog-grabber)
	CREATE_MANPAGE_PROJECT(RawLogViewer)
	CREATE_MANPAGE_PROJECT(ReactiveNavigationDemo)
	CREATE_MANPAGE_PROJECT(ReactiveNav3D-Demo)
	CREATE_MANPAGE_PROJECT(SceneViewer3D)
	CREATE_MANPAGE_PROJECT(camera-calib)
	CREATE_MANPAGE_PROJECT(icp-slam)
	CREATE_MANPAGE_PROJECT(icp-slam-live)
	CREATE_MANPAGE_PROJECT(kf-slam)
	CREATE_MANPAGE_PROJECT(2d-slam-demo)
	CREATE_MANPAGE_PROJECT(map-partition)
	CREATE_MANPAGE_PROJECT(observations2map)
	CREATE_MANPAGE_PROJECT(pf-localization)
	CREATE_MANPAGE_PROJECT(ro-localization)
	CREATE_MANPAGE_PROJECT(rbpf-slam)
	CREATE_MANPAGE_PROJECT(simul-landmarks)
	CREATE_MANPAGE_PROJECT(simul-beacons)
	CREATE_MANPAGE_PROJECT(simul-gridmap)
	CREATE_MANPAGE_PROJECT(GridmapNavSimul)
	CREATE_MANPAGE_PROJECT(grid-matching)
	CREATE_MANPAGE_PROJECT(features-matching)
	CREATE_MANPAGE_PROJECT(prrt-navigator-demo)
	CREATE_MANPAGE_PROJECT(holonomic-navigator-demo)
	CREATE_MANPAGE_PROJECT(navlog-viewer)
	CREATE_MANPAGE_PROJECT(hmt-slam)
	CREATE_MANPAGE_PROJECT(hmt-slam-gui)
	CREATE_MANPAGE_PROJECT(hmtMapViewer)
	CREATE_MANPAGE_PROJECT(track-video-features)
	CREATE_MANPAGE_PROJECT(graph-slam)
	CREATE_MANPAGE_PROJECT(graphslam-engine)
	CREATE_MANPAGE_PROJECT(kinect-3d-slam)
	CREATE_MANPAGE_PROJECT(kinect-3d-view)
	CREATE_MANPAGE_PROJECT(kinect-stereo-calib)
	CREATE_MANPAGE_PROJECT(robotic-arm-kinematics)
	CREATE_MANPAGE_PROJECT(velodyne-view)
	CREATE_MANPAGE_PROJECT(gps2rawlog)
	CREATE_MANPAGE_PROJECT(rawlog-edit)
	CREATE_MANPAGE_PROJECT(carmen2rawlog)
	CREATE_MANPAGE_PROJECT(carmen2simplemap)
	CREATE_MANPAGE_PROJECT(image2gridmap)
#	CREATE_MANPAGE_PROJECT(rawlog2carmen)
	CREATE_MANPAGE_PROJECT(ptg-configurator)

	CREATE_MANPAGE_PROJECT(DifOdometry-Datasets)
	CREATE_MANPAGE_PROJECT(DifOdometry-Camera)
	CREATE_MANPAGE_PROJECT(robot-map-gui)

	CREATE_MANPAGE_PROJECT(benchmarking-image-features)
endif (PROG_GZIP AND PROG_POD2MAN)
