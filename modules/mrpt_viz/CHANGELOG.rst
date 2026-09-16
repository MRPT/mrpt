^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_viz
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.2.0 (2026-09-16)
------------------
* fix: restore 2D overlay rendering and keep scene-camera/CText labels aligned with the expected screen-space behavior.
* refactor: modernize const-correctness and API cleanup across visualization and related containers.
* fix: resolve ROS buildfarm warnings and the correctness issues they exposed across the visualization stack.
* test: expand legacy serialization and render-buffer coverage, fixing several stale-state and rendering regressions.
* Contributors: Jose Luis Blanco-Claraco

3.1.4 (2026-09-04)
------------------

3.1.3 (2026-08-12)
------------------
* test(mrpt_viz): add extensive unit test coverage for mrpt::viz classes.
  Fix bugs in CPolyhedron init, PLY importer, and CMesh3D triangle face-normal computation.
* Contributors: Jose Luis Blanco-Claraco

3.1.2 (2026-07-07)
------------------

3.1.1 (2026-07-04)
------------------

3.1.0 (2026-07-03)
------------------
* chore: clean warnings
* Contributors: Jose Luis Blanco-Claraco

3.0.4 (2026-06-17)
------------------

3.0.3 (2026-06-15)
------------------

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

