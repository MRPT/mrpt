
# Don't declare the library if dependencies are missing:
if(NOT MRPT_ROS_VERSION EQUAL 1)
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
	NOT roscpp_FOUND
)
	message(STATUS "mrpt-ros1bridge: Skipping due to missing dependencies")
	return()
endif()

# Additional dependencies for tests only:
if (pcl_conversions_FOUND AND PCL_FOUND)
	set_property(GLOBAL PROPERTY mrpt_ros1bridge_UNIT_TEST_EXTRA_DEPS ${pcl_conversions_LIBRARIES} ${PCL_COMMON_LIBRARIES})
	set_property(GLOBAL PROPERTY mrpt_ros1bridge_UNIT_TEST_EXTRA_INCLUDE_DIRS ${pcl_conversions_INCLUDE_DIRS} ${PCL_COMMON_INCLUDE_DIRS})
	set_property(GLOBAL PROPERTY mrpt_ros1bridge_UNIT_TEST_EXTRA_DEFINITIONS "HAVE_PCL_CONVERSIONS")
endif()

define_mrpt_lib(
	ros1bridge  # Lib name
	# Dependencies:
	mrpt-maps
	mrpt-obs
	)

if(BUILD_mrpt-ros1bridge)
# Add the required libraries for linking:
	target_link_libraries(ros1bridge
		PUBLIC
		${roscpp_LIBRARIES}
		${cv_bridge_LIBRARIES}
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
			${tf2_INCLUDE_DIRS}
		PRIVATE
			${cv_bridge_INCLUDE_DIRS}
	)
	target_compile_definitions(ros1bridge PRIVATE ${ROS_DEFINITIONS})
	target_compile_definitions(ros1bridge PRIVATE IS_MRPT_ROS1BRIDGE)
endif()
