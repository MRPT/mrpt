# Check for the ROS1 libraries
# ==============================

option(DISABLE_ROS "Disable detection/usage of ROS libraries" "OFF")
mark_as_advanced(DISABLE_ROS)

if (NOT DISABLE_ROS)
	# ROS libs:
	find_package(roscpp QUIET)
	find_package(cv_bridge QUIET)
	find_package(rosbag_storage QUIET)
	find_package(pcl_conversions QUIET)

	# ROS libs for msgs:
	find_package(sensor_msgs QUIET)
	find_package(std_msgs QUIET)
	find_package(geometry_msgs QUIET)
	find_package(stereo_msgs QUIET)

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
	endif()
	# tf2_msgs: idem (header-only lib)
	find_file(tf2_msgs name TFMessage.h PATHS /usr/include/tf2_msgs)
	if (tf2_msgs)
		set(tf2_msgs_FOUND 1)
	else()
		set(tf2_msgs_FOUND 0)
	endif()
	# nav_msgs: idem (header-only lib)
	find_file(nav_msgs name OccupancyGrid.h  PATHS /usr/include/nav_msgs)
	if (nav_msgs)
		set(nav_msgs_FOUND 1)
	else()
		set(nav_msgs_FOUND 0)
	endif()

	if ($ENV{VERBOSE})
		message(STATUS "Found ROS1:")
		message(STATUS "  roscpp_INCLUDE_DIRS :${roscpp_INCLUDE_DIRS}")
		message(STATUS "  roscpp_LIBRARIES    :${roscpp_LIBRARIES}")
	endif()
endif()
