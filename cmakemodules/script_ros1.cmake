# Check for the ROS1 libraries
# ==============================

option(DISABLE_ROS "Disable detection/usage of ROS libraries" "OFF")
mark_as_advanced(DISABLE_ROS)

set(CMAKE_MRPT_HAS_ROS 0)
set(CMAKE_MRPT_HAS_ROS_SYSTEM 0)

if (NOT DISABLE_ROS)
	# Bare minimum pkg:
	find_package(roscpp QUIET)
	if(roscpp_FOUND)
		set(CMAKE_MRPT_HAS_ROS 1)
		set(CMAKE_MRPT_HAS_ROS_SYSTEM 1)

		# Optional ROS pkgs:
		find_package(std_msgs QUIET)
		find_package(sensor_msgs QUIET)
		find_package(geometry_msgs QUIET)
		find_package(nav_msgs QUIET)

		if ($ENV{VERBOSE})
			message(STATUS "Found ROS1:")
			message(STATUS "  roscpp_INCLUDE_DIRS :${roscpp_INCLUDE_DIRS}")
			message(STATUS "  roscpp_LIBRARIES    :${roscpp_LIBRARIES}")
		endif()

	endif()
endif()
