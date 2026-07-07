^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_maps
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

3.1.1 (2026-07-04)
------------------
* test(mrpt_maps): skip kdtree save/load-index tests on old nanoflann
  These tests require nanoflann >= v1.5.0's index save/load API; on
  older versions they threw and failed the suite instead of skipping.
* Contributors: Jose Luis Blanco-Claraco

3.1.0 (2026-07-03)
------------------
* Merge pull request `#1377 <https://github.com/MRPT/mrpt/issues/1377>`_ from MRPT/feat/options-capable-in-config
  feat(mrpt_config): add OptionsCapable mixin, implement in map classes
* feat(mrpt_config): add OptionsCapable mixin, implement in map classes
  Backports mola::OptionsCapable (MOLAorg/mola) into mrpt_config, next to
  CLoadableOptions: a virtual interface exposing a class' CLoadableOptions
  members generically by name, plus a safe creation-options setter. This lets
  generic tooling (de)serialize a map's insertion/likelihood/render options
  without knowing the concrete class.
  Implement it in every mrpt_maps class that holds CLoadableOptions-derived
  members: COccupancyGridMap2D, COccupancyGridMap3D, CPointsMap,
  CHeightGridMap2D, CHeightGridMap2D_MRF, CWirelessPowerGridMap2D,
  CGasConcentrationGridMap2D, CRandomFieldGridMap3D, CReflectivityGridMap2D,
  CBeaconMap, COctoMapBase and CVoxelMapOccupancyBase.
* Merge pull request `#1376 <https://github.com/MRPT/mrpt/issues/1376>`_ from MRPT/port/2x-fixes-jul2026
  Port 2.x fixes: COutputLogger thread-safety, octomap build flag
* Merge pull request `#1374 <https://github.com/MRPT/mrpt/issues/1374>`_ from MRPT/feat/kdtree-save-load-index
  feat(math): KDTreeCapable save/load of the KD-tree index (2D and 3D)
* Contributors: Jose Luis Blanco-Claraco

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

