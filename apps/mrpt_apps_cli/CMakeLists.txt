# MRPT Examples C++ - CMakeLists.txt
#/*                    _
#                     | |    Mobile Robot Programming Toolkit (MRPT)
# _ __ ___  _ __ _ __ | |_
#| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
#| | | | | | |  | |_) | |_
#|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
#               | |
#               |_|
#
# Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
# See: https://www.mrpt.org/Authors - All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#*/

cmake_minimum_required(VERSION 3.16)
project(mrpt_apps_cli)

find_package(mrpt_common REQUIRED)

# Add all CLI sub-applications (non-GUI)
add_subdirectory(carmen2rawlog)
add_subdirectory(carmen2simplemap)
add_subdirectory(gps2rawlog)
add_subdirectory(image2gridmap)
add_subdirectory(ini2yaml)
add_subdirectory(observations2map)
add_subdirectory(rawlog-edit)
add_subdirectory(rawlog-grabber)
add_subdirectory(ros-map-yaml2mrpt)
add_subdirectory(simul-beacons)
add_subdirectory(simul-gridmap)
add_subdirectory(yaml2ini)


