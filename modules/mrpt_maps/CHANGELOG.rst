^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_maps
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.1.4 (2026-09-04)
------------------
* Merge pull request `#1402 <https://github.com/MRPT/mrpt/issues/1402>`_ from MRPT/fix/restore-cpointsmapxyzirt-serialization-compat
  Restore CPointsMapXYZIRT as a deserialization-only compat shim
* tests: cover the CPointsMapXYZIRT deserialization compat stub
  Also silence the expected deprecation warning at the class registration
  site.
* Restore CPointsMapXYZIRT as a deserialization-only compat shim
  Removing the class from the RTTI registry (during the mrpt_maps module
  port) broke loading of any pre-existing .simplemap/.rawlog file that
  has a CPointsMapXYZIRT point layer: CArchive::ReadObject() throws
  "Stored object has class 'mrpt::maps::CPointsMapXYZIRT' which is not
  registered!" for any such file, since the class name is still what was
  written to disk regardless of the deprecation.
  This class carries no functionality beyond decoding the old binary
  layout (X,Y,Z + optional intensity/ring/time buffers, unchanged from
  the original implementation) into a CGenericPointsMap, so anything
  downstream that reads points via the generic field interface keeps
  working transparently. New code should use CGenericPointsMap directly;
  this class is not meant to be constructed for anything but loading old
  data.
  Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_015w31kEa1kRfKA3rcqDRXyZ
* Merge pull request `#1395 <https://github.com/MRPT/mrpt/issues/1395>`_ from MRPT/test/coverage-2026-08-31
  test(slam): raise mrpt_slam coverage to 90% and fix the bugs it uncovered
* fix(slam): report stds_Q_no_odo's heading in degrees, as it is read
  Addresses review feedback on the new TOptions dumps: CRangeBearingKFSLAM2D
  loads the third component of stds_Q_no_odo in degrees, so the dump must not
  print it in radians. Also documents that TMetricMapInitializer::saveToConfigFile()
  only round-trips the generic map params, since there is no saving counterpart
  to loadFromConfigFile_map_specific().
* test(slam): raise mrpt_slam coverage to 90% and fix the bugs it uncovered
  Adds unit tests for the parts of mrpt_slam that had none, all driven by
  synthetic data (a simulated closed room for 2D scans, a small landmark map
  for range-bearing readings) so no dataset files are needed:
  * observations_overlap and CLandmarksMap, both previously at 0%
  * CIncrementalMapPartitioner beyond the dataset test: options I/O, the three
  similarity methods, node removal, origin changes, 3D scene, serialization
  * CMetricMapBuilderICP and its CMetricMapBuilder base
  * CMetricMapBuilderRBPF / CMultiMetricMapPDF, including range-only SLAM
  * CRangeBearingKFSLAM and CRangeBearingKFSLAM2D
  * CMonteCarloLocalization2D/3D over the four particle filter algorithms
  * TSetOfMetricMapInitializers config-file round trip (in mrpt_maps)
  mrpt_slam goes from 67.9%/45.4% to 90.2%/62.3% lines/branches; the repo-wide
  figure goes from 72.5%/53.2% to 73.6%/54.4%.
  Bugs found and fixed along the way:
  * observationsOverlap()'s CSensoryFrame overload ignored its relative-pose
  argument, so the observation-overlap similarity compared keyframes as if
  co-located.
  * CIncrementalMapPartitioner::addMapFrame() passed the same relative pose for
  both directions of its symmetrized similarity; the swapped evaluation needs
  the inverse. This moves one keyframe between partitions in the existing
  dataset test, whose expectations are updated.
  * removeSetOfNodes(..., changeCoordsRef=true) composed +p instead of -p,
  doubling the first node's coordinates instead of moving it to the origin.
  * Two options were loaded with a quoted name passed to
  MRPT_LOAD_HERE_CONFIG_VAR, which stringifies it again, so they could never
  be read from a config file.
  * CLandmarksMap was never registered for RTTI and could not be deserialized.
  * TSetOfMetricMapInitializers::saveToConfigFile() wrote a format that
  loadFromConfigFile() cannot read back.
  * CMultiMetricMapPDF::getLastPose() left its is_valid_pose output untouched
  on the success path.
  * CMetricMapBuilderICP::saveCurrentEstimationToImage() dereferenced a null
  gridmap pointer right after null-checking it.
  * The range-only branch of the RBPF optimal proposal never initialized
  firstEstimateRobotHeading when odometry was present, so it always tripped
  the assert guarding it; it also printed to std::cout per particle.
  Both KF-SLAM TOptions::dumpToTextStream() now report the noise parameters
  they load instead of silently omitting them.
* Contributors: Jose Luis Blanco-Claraco

3.1.3 (2026-08-12)
------------------
* test(mrpt_maps): raise unit test coverage for occupancy grids and fix minor bugs.
* fix(mrpt_maps): make CPointsMap/CGenericPointsMap tests build on MSVC.
* test(mrpt_maps): extend coverage to octomap, voxelmap, occ3d, and multimap.
* test(mrpt_maps): raise unit-test coverage for random field grids, gas grids, and fix Voronoi clearance pointer issues.
* fix: COccupancyGridMap2D::TLikelihoodOptions::dumpToTextStream bug.
* Contributors: Jose Luis Blanco-Claraco

3.1.2 (2026-07-07)
------------------

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

