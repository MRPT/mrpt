
# Don't declare the library if dependencies are missing:
if(NOT MRPT_ROS_VERSION EQUAL 2)
	return()
endif()
if(
	NOT cv_bridge_FOUND OR
	NOT geometry_msgs_FOUND OR
	NOT nav_msgs_FOUND OR
	NOT sensor_msgs_FOUND OR
	NOT std_msgs_FOUND OR
	NOT stereo_msgs_FOUND OR
	NOT tf2_FOUND OR
	NOT rclcpp_FOUND
)
	message(STATUS "mrpt-ros2bridge: Skipping due to missing dependencies")
	return()
endif()

# Additional dependencies for tests only:
if (pcl_conversions_FOUND AND PCL_FOUND)
	set_property(GLOBAL PROPERTY mrpt_ros2bridge_UNIT_TEST_EXTRA_DEPS ${pcl_conversions_LIBRARIES} ${PCL_COMMON_LIBRARIES})
	set_property(GLOBAL PROPERTY mrpt_ros2bridge_UNIT_TEST_EXTRA_INCLUDE_DIRS ${pcl_conversions_INCLUDE_DIRS} ${PCL_COMMON_INCLUDE_DIRS})
	set_property(GLOBAL PROPERTY mrpt_ros2bridge_UNIT_TEST_EXTRA_DEFINITIONS "HAVE_PCL_CONVERSIONS")
endif()

set(EXTRA_CONFIG_CMDS
"if (NOT TARGET imp_opencv)\n\
	add_library(imp_opencv INTERFACE IMPORTED)\n\
	set_target_properties(imp_opencv\n\
		PROPERTIES\n\
		INTERFACE_INCLUDE_DIRECTORIES \"${OpenCV_INCLUDE_DIRS}\"\n\
		INTERFACE_LINK_LIBRARIES \"${OpenCV_LIBRARIES}\"\n\
		)\n\
endif()\n"
)

define_mrpt_lib(
	ros2bridge  # Lib name
	# Dependencies:
	mrpt-maps
	mrpt-obs
	# Other imported targets,
	# which will be mapped into find_package() commands
	# in the xxx_config.cmake script.
	rclcpp # find_package() lib names
	nav_msgs
	stereo_msgs
	sensor_msgs
	tf2
	cv_bridge
	)

if(BUILD_mrpt-ros2bridge)
	# Add the required libraries:
	ament_target_dependencies(ros2bridge PUBLIC
		rclcpp
		tf2
		cv_bridge
		sensor_msgs
		std_msgs
		geometry_msgs
		stereo_msgs
		nav_msgs
	)

	target_compile_definitions(ros2bridge PRIVATE ${ROS_DEFINITIONS})

	target_compile_definitions(ros2bridge PRIVATE IS_MRPT_ROS2BRIDGE)
endif()
