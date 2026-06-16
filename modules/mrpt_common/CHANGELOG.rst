^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

3.0.3 (2026-06-15)
------------------

3.0.2 (2026-06-11)
------------------
* fix: BUILD_TESTING=OFF default was ignored due to include(CTest) ordering
  CTest's own option(BUILD_TESTING ... ON) ran first and created the cache
  variable, making our subsequent option(BUILD_TESTING ... OFF) a no-op.
  This left BUILD_TESTING=ON on the ROS build farm (which doesn't pass
  -DBUILD_TESTING=OFF), causing find_package(GTest REQUIRED) to fail and
  break the mrpt_core binary build.
  Also fix CI workflows to pass the correct -DBUILD_TESTING=ON flag
  (macOS/Windows previously used the non-existent MRPT_BUILD_TESTING,
  and Linux relied on the buggy default).
* Contributors: Jose Luis Blanco-Claraco

3.0.1 (2026-06-11)
------------------
* Merge pull request `#1363 <https://github.com/MRPT/mrpt/issues/1363>`_ from MRPT/fix/dont-export-eigen3-dep
  refactor: limit visibility of eigen3 as build dep
* refactor: limit visibility of eigen3 as build dep
* cmake: default BUILD_TESTING to OFF to fix FTBFS on Humble build farm
* Contributors: Jose Luis Blanco-Claraco

3.0.0 (2026-06-06)
------------------

2.20.0 (2026-06-06)
--------------------
* Last release of the 2.x series. Starting from 3.0.0, changes are tracked
  in each module's own CHANGELOG.rst file.
* Contributors: Jose Luis Blanco-Claraco

0.4.0 (2024-08-20)
------------------
* Reorganize cmake scripts to make them compatible with both ROS1 catkin and ROS2 ament
* Contributors: Jose Luis Blanco-Claraco

0.3.3 (2024-08-14)
------------------
* make the package to build on ROS 1 too
* Contributors: Jose Luis Blanco-Claraco

0.3.2 (2024-08-09)
------------------
* Fix ament_xmllint warnings
* Contributors: Jose Luis Blanco-Claraco

0.3.1 (2024-04-30)
------------------
* Bump cmake_minimum_required to 3.5
* Fix clang warning
* Contributors: Jose Luis Blanco-Claraco

0.3.0 (2024-01-07)
------------------
* Fix usage of mola:: cmake prefix
* add package file attribute
* Contributors: Jose Luis Blanco-Claraco

0.2.2 (2023-09-08)
------------------
* Fix package name in docs
* Generate ament-correct package for ROS2 builds
* fix lib name in cmake warning message
* Contributors: Jose Luis Blanco-Claraco

0.2.1 (2023-09-02)
------------------

0.2.0 (2023-08-24)
------------------
* First public release as ROS 2 package.
