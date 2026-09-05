^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_obs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.1.4 (2026-09-04)
------------------
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
* fix: TOPCON doc sign convention, test state leak.
* fix(mrpt_obs): add missing explicit template instantiations for TPixelLabelInfo stream I/O.
* mrpt_obs: increase code coverage and fix bugs across multiple observation classes.
  Fix bugs in CSensoryFrame erase, Velodyne YAML loading, GNSS message types, 3D range scan loading, and 2D scan conversion.
* Contributors: Jose Luis Blanco-Claraco

3.1.2 (2026-07-07)
------------------

3.1.1 (2026-07-04)
------------------

3.1.0 (2026-07-03)
------------------

3.0.4 (2026-06-17)
------------------

3.0.3 (2026-06-15)
------------------
* Fix rawlog-edit CLI unittest failure on Windows CI (`#1370 <https://github.com/MRPT/mrpt/issues/1370>`_)
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

