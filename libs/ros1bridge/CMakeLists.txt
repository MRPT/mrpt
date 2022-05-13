
# Don't declare the library if dependencies are missing:
if(NOT CMAKE_MRPT_ROS_VERSION EQUAL 1)
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
if (PCL_FOUND)
	set_property(GLOBAL PROPERTY mrpt_ros1bridge_UNIT_TEST_EXTRA_DEPS "${PCL_COMMON_LIBRARIES}")
	set_property(GLOBAL PROPERTY mrpt_ros1bridge_UNIT_TEST_EXTRA_DEFINITIONS "HAVE_PCL")
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
		roscpp::roscpp
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
		${cv_bridge_INCLUDE_DIRS}
	)
	target_compile_definitions(ros1bridge PRIVATE ${ROS_DEFINITIONS})
	target_compile_definitions(ros1bridge PRIVATE IS_MRPT_ROS1BRIDGE)
endif()
