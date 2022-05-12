
# Don't declare the library if dependencies are missing:
if(NOT CMAKE_MRPT_ROS_VERSION EQUAL 2)
	return()
endif()
if(
	NOT cv_bridge_FOUND OR
	NOT geometry_msgs_FOUND OR
	NOT nav_msgs_FOUND OR
	NOT sensor_msgs_FOUND OR
	NOT std_msgs_FOUND OR
	NOT stereo_msgs_FOUND OR
	NOT tf2_FOUND
)
	message(STATUS "mrpt-ros2bridge: Skipping due to missing dependencies")
	return()
endif()


define_mrpt_lib(
	ros2bridge  # Lib name
	# Dependencies:
	mrpt-maps
	mrpt-obs
	)

if(BUILD_mrpt-ros2bridge)
	if (pcl_conversions_FOUND)
		target_compile_definitions(ros2bridge PUBLIC MRPT_HAVE_PCL_CONVERSIONS=1)
	else()
		target_compile_definitions(ros2bridge PUBLIC MRPT_HAVE_PCL_CONVERSIONS=0)
	endif()

	# Add the required libraries for linking:
	target_link_libraries(ros2bridge
		PUBLIC
		${roscpp_LIBRARIES}
		${rosbag_storage_LIBRARIES}
		${cv_bridge_LIBRARIES}
		${pcl_conversions_LIBRARIES}
	)
	if (tf2_LIBRARIES)
		target_link_libraries(ros2bridge PUBLIC ${tf2_LIBRARIES})
	endif()
	if (TARGET tf2)
		target_link_libraries(ros2bridge PUBLIC tf2)
	endif()

	target_include_directories(ros2bridge
		SYSTEM
		PUBLIC
		#${roscpp_INCLUDE_DIRS}
		#${rosbag_storage_INCLUDE_DIRS}
		${cv_bridge_INCLUDE_DIRS}
		${pcl_conversions_INCLUDE_DIRS}
		${tf2_INCLUDE_DIRS}
	)
	target_compile_definitions(ros2bridge PRIVATE ${ROS_DEFINITIONS})
	target_compile_definitions(ros2bridge PRIVATE IS_MRPT_ROS2BRIDGE)

endif()
