^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_slam
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.1.4 (2026-09-04)
------------------
* Merge pull request `#1395 <https://github.com/MRPT/mrpt/issues/1395>`_ from MRPT/test/coverage-2026-08-31
  test(slam): raise mrpt_slam coverage to 90% and fix the bugs it uncovered
* test(slam): give simulated SLAM steps explicit, increasing timestamps
  Clock::now() has a ~15 ms resolution on Windows, so calling it once per
  simulated step returned the same timestamp for consecutive odometry actions.
  CRobot2DPoseEstimator drops updates whose timestamp does not advance, so the
  ICP map builder's pose estimate never moved and the test failed there.
  Timestamps now come from a helper that hands out values 100 ms apart, with the
  action and the observation of each step sharing the same one.
* test(slam): do not depend on the order of the spectral partitions
  The two clusters returned by RecursiveSpectralPartition() are stable, but
  which one comes first depends on the sign of the eigenvector and differs
  between platforms (the macOS CI job returned them swapped). Compare them in a
  canonical order instead.
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
* Merge pull request `#1389 <https://github.com/MRPT/mrpt/issues/1389>`_ from MRPT/test/coverage-graphslam-system-io
  test(graphslam, system, io, slam): cover the largest untested pure-logic files, fix 8 bugs
* test(graphslam,system,io,slam): cover the largest untested pure-logic files
  Second coverage pass, on the four biggest pure-logic gaps left after
  mrpt_nav/mrpt_kinematics:
  mrpt_graphslam  41.0% -> 72.5% lines (33.0% -> 59.6% branches)
  mrpt_slam       64.1% -> 67.5%       (43.1% -> 44.9%)
  mrpt_io         57.0% -> 63.6%       (40.2% -> 46.4%)
  mrpt_system     54.4% -> 61.7%       (40.5% -> 41.9%)
  Five files that had never had a single assertion run against them are now
  covered: TSlidingWindow.cpp, CEdgeCounter.cpp, md5.cpp,
  CRejectionSamplingRangeOnlyLocalization.cpp (all 0%) and
  vector_loadsave.cpp (16%).
  Bugs found and fixed, each reproduced by a failing test first:
  * TSlidingWindow::getMean() divided by zero on an empty window and
  returned NaN. Since every comparison against a NaN is false,
  evaluateMeasurementAbove() was then stuck at false for any input.
  * TSlidingWindow::getStdDev() normalized by the window capacity instead
  of the number of measurements held, so a partially-filled window
  systematically under-reported sigma -- and disagreed with getMean(),
  which uses the sample count.
  * TSlidingWindow::resizeWindow() invalidated the mean and median caches
  but never the std-dev one: stale after a shrink, and on a grow it
  invalidated nothing at all even though the value depends on the window
  size.
  * CEdgeCounter::clearAllEdges() reset every counter except
  m_unique_edges, so a cleared instance still reported a stale
  unique-edge total.
  * mrpt::system::md5(const std::vector<uint8_t>&) used &str[0], an
  out-of-bounds access for an empty vector whose resulting pointer then
  tripped the ASSERT\_(data) of the overload it delegates to: md5() of an
  empty vector threw, while md5() of an empty string returned the
  correct digest. The algorithm itself passes every RFC 1321 vector.
  * mrpt::io::vectorNumericFromTextFile() had three defects: it discarded
  fscanf()'s return value in its default byRows==false path (the
  (!byRows) || short-circuits), so a failed read still pushed the stale
  value and an empty file yielded {0.0}; it never cleared its output
  vector, unlike loadTextFile()/loadBinaryFile(); and it leaked the FILE
  handle that every sibling function in the same file closes.
  CRejectionSamplingRangeOnlyLocalization was the largest 0% file in the
  repo but its 10 new tests all passed first time: it was untested, not
  broken.
  agents.md records the new numbers, the two behaviors left documented
  rather than changed, and a gotcha about the reproduce recipe: it excludes
  apps/, so mrpt_libapps_cli's ~40 CLI tests silently skip and the module
  measures ~8% instead of ~59%.
* Contributors: Jose Luis Blanco-Claraco

3.1.3 (2026-08-12)
------------------

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

