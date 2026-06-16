^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_apps_gui
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.4 (2026-06-17)
------------------

3.0.3 (2026-06-15)
------------------

3.0.2 (2026-06-11)
------------------

3.0.1 (2026-06-11)
------------------
* fix: clang-format include order in perf-graphslam.cpp
* fix: enable isolated/Debian-package builds of apps_gui
  Two issues surface when building packages in isolation (as the ROS build farm
  and .deb packaging do), where sibling modules are only available as installed
  packages, not as source:
  - mrpt-performance/perf-graphslam.cpp reached into another module's test
  sources via a relative path (../../../modules/mrpt_graphslam/tests/...).
  Vendor a local copy of the small test helper header instead.
  - mrpt_gui installs the public header nanogui/opengl.h which includes
  <GLFW/glfw3.h>, but only declared libglfw3-dev as a build dependency. Make it
  a public <depend> so downstream consumers get the GLFW headers.
* Merge pull request `#1363 <https://github.com/MRPT/mrpt/issues/1363>`_ from MRPT/fix/dont-export-eigen3-dep
  refactor: limit visibility of eigen3 as build dep
* fix: missing include
* fix: KF math errors
* fix: build errors from limiting Eigen3 visibility + KF Joseph form
  Fix CI build failures introduced by making Eigen3 a private build
  dependency:
  - 2d-slam-demo: drop unnecessary Eigen::aligned_allocator from
  m_historicData; a plain std::vector suffices and no longer needs
  Eigen headers transitively.
  - topography_gps_coords_example: this example genuinely uses MRPT
  matrix methods that require <Eigen/Dense> in the calling TU, so
  link Eigen3::Eigen explicitly instead of relying on transitive
  exposure.
  Also fix a correctness bug in the new dense Joseph-form covariance
  update: it added K*S*K^T (S = H*P*H^T + R), double-counting the
  H*P*H^T*K^T term already present in (I-K*H)*P*(I-K*H)^T. Use the
  measurement noise K*R*K^T instead, matching the algebraically
  correct sparse update path.
* Contributors: Jose Luis Blanco-Claraco

3.0.0 (2026-06-06)
------------------

2.20.0 (2026-06-06)
-------------------
* Last release of the 2.x series. Starting from 3.0.0, changes are tracked
  in each module's own CHANGELOG.rst file.
* Contributors: Jose Luis Blanco-Claraco

