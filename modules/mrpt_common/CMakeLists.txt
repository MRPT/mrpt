# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
#
# Copyright (C) 2018-2025, Jose Luis Blanco-Claraco, contributors (AUTHORS.md)
# All rights reserved.
# Released under BSD-3. See LICENSE file
# ------------------------------------------------------------------------------

# Minimum CMake version: deprecated if <3.5 (as of Apr 2024)
cmake_minimum_required(VERSION 3.5)

if("$ENV{ROS_VERSION}" STREQUAL "2")
	set(DETECTED_ROS2 TRUE)
	set(PACKAGE_ROS_VERSION 2)
elseif("$ENV{ROS_VERSION}" STREQUAL "1")
	set(DETECTED_ROS1 TRUE)
	set(PACKAGE_ROS_VERSION 1)
	set(CMAKE_CXX_STANDARD 14)
endif()

project(mola_common)

include(GNUInstallDirs) # for install dirs in multilib
include(CMakePackageConfigHelpers)

# Instrumental lib:
add_library(${PROJECT_NAME} INTERFACE)

# Project version:
include(mola-version.cmake)

# Allow build-time finding of this package:
set(mola_common_DIR ${CMAKE_CURRENT_SOURCE_DIR} CACHE PATH "Path to mola_common")

# not ament-based builds (standalone, w/o ROS):
if ((NOT DETECTED_ROS2) AND (NOT DETECTED_ROS1))
	# Generate: mola_common-version.cmake
	include(CMakePackageConfigHelpers)

	if(NOT ${CMAKE_VERSION} VERSION_LESS "3.14.0")
		set(ARCH_INDEP_FLAG "ARCH_INDEPENDENT")
	endif()

	write_basic_package_version_file(
		# generate to the topmost root binary dir:
		${CMAKE_BINARY_DIR}/mola_common-config-version.cmake
		VERSION ${MOLA_VERSION_NUMBER_MAJOR}.${MOLA_VERSION_NUMBER_MINOR}.${MOLA_VERSION_NUMBER_PATCH}
		COMPATIBILITY AnyNewerVersion
		${ARCH_INDEP_FLAG}
	)

	# Generate: mola_common.cmake
	configure_file(
		# Input:
		mola_common-config.cmake.in
		# Output: generate to the topmost root binary dir:
		${CMAKE_BINARY_DIR}/mola_common-config.cmake
		# Parse: "@var_names@" only
		@ONLY
	)

	# Install files:
	install(
		FILES
			${CMAKE_BINARY_DIR}/${PROJECT_NAME}-config.cmake
			${CMAKE_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
			${CMAKE_SOURCE_DIR}/mola-version.cmake
			DESTINATION
				share/${PROJECT_NAME}/cmake
	)

	# Provide make uninstall:
	if (NOT TARGET uninstall)
	configure_file(
		"${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
		"${CMAKE_BINARY_DIR}/cmake_uninstall.cmake"
		IMMEDIATE @ONLY)
	add_custom_target(uninstall
		COMMAND ${CMAKE_COMMAND} -P ${CMAKE_BINARY_DIR}/cmake_uninstall.cmake)
	endif()
endif()


# -----------------------------------------------------------------------------
#  common
# -----------------------------------------------------------------------------
set(CUSTOM_CMAKE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Install files common to both, ROS and non-ROS builds:
install(
	FILES
		${CUSTOM_CMAKE_DIR}/mola_cmake_functions.cmake
		${CUSTOM_CMAKE_DIR}/mola-xxx-config.cmake.in
		DESTINATION
			share/${PROJECT_NAME}/cmake
)


# -----------------------------------------------------------------------------
#  ROS1
# -----------------------------------------------------------------------------
if (DETECTED_ROS1)
	# Find the required catkin packages
	find_package(catkin REQUIRED)

	# Declare this project as a catkin package
	catkin_package(
		CFG_EXTRAS mola_cmake_functions.cmake
	)
endif()

# -----------------------------------------------------------------------------
#  ROS2
# -----------------------------------------------------------------------------
if (DETECTED_ROS2)
	# find dependencies
	find_package(ament_cmake REQUIRED)

	#ament_target_dependencies(MY_TARGET
	#  rclcpp...
	#)

	# Add custom mola cmake macros, as in our non-ament (ROS2) config.cmake file:
	# See: mola_common-config.cmake.in

	ament_package(
		CONFIG_EXTRAS cmake/mola_cmake_functions.cmake
	)
endif()
