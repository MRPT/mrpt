# Check for the ROS1 libraries
# ==============================

option(DISABLE_ROS "Disable detection/usage of ROS libraries" "OFF")
mark_as_advanced(DISABLE_ROS)

if (NOT DISABLE_ROS)
	# Bare minimum pkg:
	find_package(roscpp QUIET)
	find_package(sensor_msgs QUIET)
	find_package(cv_bridge QUIET)
	find_package(rosbag QUIET)
	find_package(std_msgs QUIET)
	find_package(geometry_msgs QUIET)

	# These ones fail due to use of find_package() instead of find_dependency()
	# internally, so check first if we have a full ROS distro or Debian-based
	# pkgs only:
	find_package(catkin QUIET)
	if (catkin_FOUND)
		find_package(tf2_msgs QUIET)
		find_package(tf2 QUIET)
		find_package(nav_msgs QUIET)
	endif()

	if ($ENV{VERBOSE})
		message(STATUS "Found ROS1:")
		message(STATUS "  roscpp_INCLUDE_DIRS :${roscpp_INCLUDE_DIRS}")
		message(STATUS "  roscpp_LIBRARIES    :${roscpp_LIBRARIES}")
	endif()
endif()
