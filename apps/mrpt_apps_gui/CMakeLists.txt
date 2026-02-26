# MRPT GUI Applications - CMakeLists.txt
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
project(mrpt_apps_gui)

find_package(mrpt_common REQUIRED)

# Add GUI sub-applications (ported from old monolithic build)
add_subdirectory(graph-slam)
add_subdirectory(graphslam-engine)
add_subdirectory(grid-matching)
add_subdirectory(icp-slam)
add_subdirectory(icp-slam-live)
add_subdirectory(kf-slam)
add_subdirectory(map-partition)
add_subdirectory(mrpt-performance)
add_subdirectory(pf-localization)
add_subdirectory(rbpf-slam)
add_subdirectory(ro-localization)
add_subdirectory(simul-landmarks)

# Legacy GUI applications (to be ported)
# Uncomment and add legacy apps as they are ported to follow the new pattern:
# add_subdirectory(2d-slam-demo)
# add_subdirectory(3d-rotation-converter)
# ... etc
