
# Don't declare the library if dependencies are missing:
if(
	NOT cv_bridge_FOUND OR
	NOT geometry_msgs_FOUND OR
	NOT nav_msgs_FOUND OR
	(NOT rosbag_storage_FOUND AND NOT rosbag2_FOUND) OR
	NOT sensor_msgs_FOUND OR
	NOT std_msgs_FOUND OR
	NOT stereo_msgs_FOUND OR
	NOT tf2_FOUND
)
	message(STATUS "mrpt-ros1bridge: Skipping due to missing dependencies")
	return()
endif()


define_mrpt_lib(
	ros1bridge  # Lib name
	# Dependencies:
	mrpt-maps
	mrpt-obs
	)

if(BUILD_mrpt-ros1bridge)
	if (pcl_conversions_FOUND)
		target_compile_definitions(ros1bridge PUBLIC MRPT_HAVE_PCL_CONVERSIONS=1)
	else()
		target_compile_definitions(ros1bridge PUBLIC MRPT_HAVE_PCL_CONVERSIONS=0)
	endif()

	# Add the required libraries for linking:
	target_link_libraries(ros1bridge
		PUBLIC
		${roscpp_LIBRARIES}
		${rosbag_storage_LIBRARIES}
		${cv_bridge_LIBRARIES}
		${pcl_conversions_LIBRARIES}
	)
	if (tf2_LIBRARIES)
		target_link_libraries(ros1bridge PUBLIC ${tf2_LIBRARIES})
	endif()
	if (TARGET tf2)
		target_link_libraries(ros1bridge PUBLIC tf2)
	endif()

	target_include_directories(ros1bridge
		SYSTEM
		PUBLIC
		${roscpp_INCLUDE_DIRS}
		${rosbag_storage_INCLUDE_DIRS}
		${cv_bridge_INCLUDE_DIRS}
		${pcl_conversions_INCLUDE_DIRS}
		${tf2_INCLUDE_DIRS}
	)
	target_compile_definitions(ros1bridge PRIVATE ${ROS_DEFINITIONS})

endif()
