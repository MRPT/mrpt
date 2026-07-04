^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_opengl
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

3.1.0 (2026-07-03)
------------------

3.0.4 (2026-06-17)
------------------
* fix: conservative use depend to ensure opengl binary libs are added downstream of mrpt_opengl
* Merge pull request `#1371 <https://github.com/MRPT/mrpt/issues/1371>`_ from wentasah/export-opengl
  mrpt_opengl: Add opengl build_depend back
* mrpt_opengl: Add opengl build_depend back
  In a recent commit, build_depend was replaced with
  build_export_depend. But it seems that both build_depend and
  build_export_depend need to be specified. Without build_depend, ROS
  build farm complains about OpenGL not being available:
  Could NOT find OpenGL (missing: OPENGL_opengl_LIBRARY OPENGL_glx_LIBRARY OPENGL_INCLUDE_DIR)
* Contributors: Jose Luis Blanco-Claraco, Michal Sojka

3.0.3 (2026-06-15)
------------------
* Merge pull request `#1368 <https://github.com/MRPT/mrpt/issues/1368>`_ from wentasah/export-opengl
  mrpt_opengl: Export opengl dependency
* Contributors: Jose Luis Blanco-Claraco, Michal Sojka

3.0.2 (2026-06-11)
------------------

3.0.1 (2026-06-11)
------------------
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

