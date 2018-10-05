include(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

project(rawlog-edit)

# ================================================
# TARGET: rawlog-edit
# ================================================
# Define the executable target:
add_executable(${PROJECT_NAME}
	# Main:
	rawlog-edit_main.cpp
	# Headers
	rawlog-edit-declarations.h
	CRawlogProcessor.h
	# Ops:
	rawlog-edit_list-poses.cpp
	rawlog-edit_gps.cpp
	rawlog-edit_gps_gas.cpp
	rawlog-edit_info.cpp
	rawlog-edit_externalize.cpp
	rawlog-edit_filters.cpp
	rawlog-edit_cuts.cpp
	rawlog-edit_rawdaq.cpp
	rawlog-edit_sensor-poses.cpp
	rawlog-edit_camera-params.cpp
	rawlog-edit_list-images.cpp
	rawlog-edit_list-rangbear.cpp
	rawlog-edit_generate-3d-pointclouds.cpp
	rawlog-edit_generate-pcd.cpp
	rawlog-edit_stereo-rectify.cpp
	rawlog-edit_rename_externals.cpp
	rawlog-edit_list-timestamps.cpp
	rawlog-edit_remap_timestamps.cpp
	rawlog-edit_imu.cpp
	rawlog-edit_2d-scans.cpp
	rawlog-edit_odometry.cpp
	rawlog-edit_enose.cpp
	rawlog-edit_anemometer.cpp
	${MRPT_VERSION_RC_FILE}
 	)

# Add the required libraries for linking:
target_link_libraries(${PROJECT_NAME} ${MRPT_LINKER_LIBS})
DeclareAppForInstall(${PROJECT_NAME})
# Dependencies on MRPT libraries: Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${PROJECT_NAME} mrpt-obs mrpt-topography mrpt-maps mrpt-vision mrpt-tclap)
