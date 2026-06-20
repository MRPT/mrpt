# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MRPT)
#
# Copyright (C) 2018-2025, Jose Luis Blanco-Claraco, contributors (AUTHORS.md)
# All rights reserved.
# Released under BSD-3. See LICENSE file
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.16)

if("$ENV{ROS_VERSION}" STREQUAL "2")
	set(DETECTED_ROS2 TRUE)
	set(PACKAGE_ROS_VERSION 2)
elseif("$ENV{ROS_VERSION}" STREQUAL "1")
	set(DETECTED_ROS1 TRUE)
	set(PACKAGE_ROS_VERSION 1)
	set(CMAKE_CXX_STANDARD 14)
endif()

project(mrpt_common)

include(GNUInstallDirs) # for install dirs in multilib
include(CMakePackageConfigHelpers)

# Instrumental lib:
add_library(${PROJECT_NAME} INTERFACE)

#----
# Extract version from package.xml
# Example line:" <version>0.3.2</version>"
file(READ package.xml contentPackageXML)
string(REGEX MATCH "<version>([0-9\.]*)</version>" _ ${contentPackageXML})
set(MRPT_VERSION ${CMAKE_MATCH_1})
message(STATUS "MRPT version: ${MRPT_VERSION} (detected in package.xml)")
unset(contentPackageXML)
string(REGEX MATCH "^([0-9]+)\\.([0-9]+)\\.([0-9]+)" _ ${MRPT_VERSION})
set(MRPT_MAJOR_VERSION ${CMAKE_MATCH_1})
set(MRPT_MINOR_VERSION ${CMAKE_MATCH_2})
set(MRPT_PATCH_VERSION ${CMAKE_MATCH_3})
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/mrpt_common-version.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/mrpt_common-version.cmake
    @ONLY
)
#----


# Allow build-time finding of this package:
set(mrpt_common_DIR ${CMAKE_CURRENT_SOURCE_DIR} CACHE PATH "Path to mrpt_common")

# not ament-based builds (standalone, w/o ROS):
if ((NOT DETECTED_ROS2) AND (NOT DETECTED_ROS1))
	# Generate: mrpt_common-version.cmake
	include(CMakePackageConfigHelpers)

	if(NOT ${CMAKE_VERSION} VERSION_LESS "3.14.0")
		set(ARCH_INDEP_FLAG "ARCH_INDEPENDENT")
	endif()

	write_basic_package_version_file(
		# generate to the topmost root binary dir:
		${CMAKE_BINARY_DIR}/mrpt_common-config-version.cmake
		VERSION ${MRPT_MAJOR_VERSION}.${MRPT_MINOR_VERSION}.${MRPT_PATCH_VERSION}
		COMPATIBILITY AnyNewerVersion
		${ARCH_INDEP_FLAG}
	)

	# Generate: mrpt_common.cmake
	configure_file(
		# Input:
		mrpt_common-config.cmake.in
		# Output: generate to the topmost root binary dir:
		${CMAKE_BINARY_DIR}/mrpt_common-config.cmake
		# Parse: "@var_names@" only
		@ONLY
	)

	# Install files:
	install(
		FILES
			${CMAKE_BINARY_DIR}/${PROJECT_NAME}-config.cmake
			${CMAKE_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
			${CMAKE_BINARY_DIR}/mrpt_common-version.cmake
			DESTINATION
				share/${PROJECT_NAME}/cmake
	)
	install(
		DIRECTORY
			${CMAKE_SOURCE_DIR}/common_sources
			${CMAKE_SOURCE_DIR}/common_headers
			DESTINATION
				share/${PROJECT_NAME}/
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
# Install files common to both, ROS and non-ROS builds:
install(
	FILES
		${CMAKE_CURRENT_SOURCE_DIR}/cmake/mrpt_cmake_functions.cmake
		${CMAKE_CURRENT_SOURCE_DIR}/cmake/mrpt-xxx-config.cmake.in
		DESTINATION
			share/${PROJECT_NAME}/cmake
)
install(
	DIRECTORY
		${CMAKE_CURRENT_SOURCE_DIR}/common_headers
		${CMAKE_CURRENT_SOURCE_DIR}/common_sources
		DESTINATION
			share/${PROJECT_NAME}/
)

# -----------------------------------------------------------------------------
#  ROS1
# -----------------------------------------------------------------------------
if (DETECTED_ROS1)
	# Find the required catkin packages
	find_package(catkin REQUIRED)

	# Declare this project as a catkin package
	catkin_package(
		CFG_EXTRAS mrpt_cmake_functions.cmake
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

	# Add custom mrpt cmake macros, as in our non-ament (ROS2) config.cmake file:
	# See: mrpt_common-config.cmake.in

	ament_package(
		CONFIG_EXTRAS cmake/mrpt_cmake_functions.cmake
	)
endif()

# Silent cmake warnings:
set(dummy ${CMAKE_EXPORT_COMPILE_COMMANDS})
unset(dummy)

