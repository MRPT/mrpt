^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_gui
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.1.0 (2026-07-03)
------------------

3.0.4 (2026-06-17)
------------------

3.0.3 (2026-06-15)
------------------

3.0.2 (2026-06-11)
------------------

3.0.1 (2026-06-11)
------------------
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
* refactor: limit visibility of eigen3 as build dep
* Contributors: Jose Luis Blanco-Claraco

3.0.0 (2026-06-06)
------------------

2.20.0 (2026-06-06)
-------------------
* Last release of the 2.x series. Starting from 3.0.0, changes are tracked
  in each module's own CHANGELOG.rst file.
* Contributors: Jose Luis Blanco-Claraco

