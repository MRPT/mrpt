# Check for the ROS1 libraries
# ==============================

option(DISABLE_ROS "Disable detection/usage of ROS libraries" "OFF")
mark_as_advanced(DISABLE_ROS)

set(MRPT_ROS_VERSION ) # empty=none

if (NOT DISABLE_ROS)

	# Save flag:
	get_directory_property(FORMER_DEFS "COMPILE_DEFINITIONS")
	if (NOT "${FORMER_DEFS}" STREQUAL "")
	  message(FATAL_ERROR "Expected: empty COMPILE_DEFINITIONS at this point")
	endif()

	# ROS libs:
	find_package(ament_cmake QUIET)
	if(ament_cmake_FOUND)
		set(MRPT_ROS_VERSION 2)
	else()
		find_package(roscpp QUIET)
		if (roscpp_FOUND)
			set(MRPT_ROS_VERSION 1)
		endif()
	endif()

	# ======================================
	# Search for common deps in ROS1 & ROS2
	# ======================================
	find_package(cv_bridge QUIET)
	find_package(rosbag_storage QUIET)
	find_package(rosbag2 QUIET)

	# optional, for tests only:
	find_package(pcl_conversions QUIET)
	find_package(PCL QUIET COMPONENTS common)
	
	# ROS libs for msgs:
	find_package(sensor_msgs QUIET)
	find_package(std_msgs QUIET)
	find_package(geometry_msgs QUIET)
	find_package(stereo_msgs QUIET)

	if ("${MRPT_ROS_VERSION}" STREQUAL "1")
		# =================================
		# Search for deps in ROS1 style
		# =================================

		# tf2: try to find w/o the need for catkin (so we can use it from
		# Debian build servers w/o ROS installed under /opt/ etc.)
		# (JLBC) For some reason calling find_package(tf2) leads to error
		# in that case due to missing catkin stuff, while the lib & .h's are
		# actually there and are usable.
		find_library(tf2_LIBRARIES tf2)
		if (tf2_LIBRARIES)
			set(tf2_FOUND 1)
		else()
			set(tf2_FOUND 0)
			# 2nd attempt via cmake:
			unset(tf2_LIBRARIES CACHE)
			unset(tf2_INCLUDE_DIRS CACHE)
			find_package(tf2 QUIET)
		endif()
		# tf2_msgs: idem (header-only lib)
		find_file(tf2_msgs name TFMessage.h
			PATHS
				/usr/include/tf2_msgs
				$ENV{ROS_ROOT}/../../include/tf2_msgs
		)
		if (tf2_msgs)
			set(tf2_msgs_FOUND 1)
		else()
			set(tf2_msgs_FOUND 0)
			# 2nd attempt via cmake:
			unset(tf2_msgs_LIBRARIES CACHE)
			unset(tf2_msgs_INCLUDE_DIRS CACHE)
			find_package(tf2_msgs QUIET)
		endif()
		# nav_msgs: idem (header-only lib)
		find_file(nav_msgs name OccupancyGrid.h
			PATHS
				/usr/include/nav_msgs
				$ENV{ROS_ROOT}/../../include/nav_msgs
		)
		if (nav_msgs)
			set(nav_msgs_FOUND 1)
		else()
			set(nav_msgs_FOUND 0)
			# 2nd attempt via cmake:
			unset(nav_msgs_LIBRARIES CACHE)
			unset(nav_msgs_INCLUDE_DIRS CACHE)
			find_package(nav_msgs QUIET)
		endif()
	elseif("${MRPT_ROS_VERSION}" STREQUAL "2")
		# =================================
		# Search for deps in ROS2 style
		# =================================
		find_package(rclcpp QUIET)
		find_package(tf2 QUIET)
		find_package(tf2_msgs QUIET)
		find_package(nav_msgs QUIET)
	endif()

	# Compare flag:
	get_directory_property(ROS_DEFINITIONS "COMPILE_DEFINITIONS")
	set_directory_properties(PROPERTIES "COMPILE_DEFINITIONS" "")


	if ("$ENV{VERBOSE}")
		message(STATUS "ROS dependencies:")
		message(STATUS "  roscpp_INCLUDE_DIRS :${roscpp_INCLUDE_DIRS}")
		message(STATUS "  roscpp_LIBRARIES    :${roscpp_LIBRARIES}")
		message(STATUS "  ament_cmake_INCLUDE_DIRS :${ament_cmake_INCLUDE_DIRS}")
		message(STATUS "  ament_cmake_LIBRARIES    :${ament_cmake_LIBRARIES}")
		message(STATUS "  rosbag_storage_INCLUDE_DIRS :${rosbag_storage_INCLUDE_DIRS}")
		message(STATUS "  rosbag_storage_LIBRARIES    :${rosbag_storage_LIBRARIES}")
		message(STATUS "  cv_bridge_INCLUDE_DIRS :${cv_bridge_INCLUDE_DIRS}")
		message(STATUS "  cv_bridge_LIBRARIES    :${cv_bridge_LIBRARIES}")
		message(STATUS "  tf2_INCLUDE_DIRS :${tf2_INCLUDE_DIRS}")
		message(STATUS "  tf2_LIBRARIES    :${tf2_LIBRARIES}")
		message(STATUS "  ROS definitions     :${ROS_DEFINITIONS}")
		message(STATUS "  rclcpp_FOUND:        : ${rclcpp_FOUND}")
		message(STATUS "  cv_bridge_FOUND      : ${cv_bridge_FOUND}")
		message(STATUS "  geometry_msgs_FOUND  : ${geometry_msgs_FOUND}")
		message(STATUS "  nav_msgs_FOUND       : ${nav_msgs_FOUND}")
		message(STATUS "  rosbag_storage_FOUND : ${rosbag_storage_FOUND}")
		message(STATUS "  rosbag2_FOUND        : ${rosbag2_FOUND}")
		message(STATUS "  sensor_msgs_FOUND    : ${sensor_msgs_FOUND}")
		message(STATUS "  std_msgs_FOUND       : ${std_msgs_FOUND}")
		message(STATUS "  stereo_msgs_FOUND    : ${stereo_msgs_FOUND}")
		message(STATUS "  tf2_FOUND            : ${tf2_FOUND}")
	endif()
endif()
