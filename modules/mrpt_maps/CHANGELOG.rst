^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_maps
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.4 (2026-06-17)
------------------

3.0.3 (2026-06-15)
------------------
* Fix rawlog-edit CLI unittest failure on Windows CI (`#1370 <https://github.com/MRPT/mrpt/issues/1370>`_)
* Update nanoflann to 1.10.1 and add KD-tree query tests for CPointsMap
* Merge pull request `#1366 <https://github.com/MRPT/mrpt/issues/1366>`_ from MRPT/feature/occgrid2d-test-coverage-completion
  test(mrpt_maps): complete COccupancyGridMap2D unit test coverage
* test(mrpt_maps): add COccupancyGridMap2D coverage for subSample, simulate, likelihood, matching
  Adds a subSampleHalvesResolution unit test and three new test files covering
  laser/sonar scan simulation, all TLikelihoodMethod likelihood computations,
  and determineMatching2D.
* Contributors: Jose Luis Blanco-Claraco

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

