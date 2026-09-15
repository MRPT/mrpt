^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_obs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge pull request `#1413 <https://github.com/MRPT/mrpt/issues/1413>`_ from MRPT/feat/api-cleanups-3.1
  feat: API cleanups before the next minor bump (archives, const-correctness, points-map accessor)
* feat(mrpt_obs,mrpt_viz,mrpt_maps): deep const-correctness in smart-pointer containers
  Reading through a `const` container of `X::Ptr` handed out mutable
  pointees, so constness stopped at the container. Add a small
  `mrpt::containers::deep_const_iterator` proxy and use it for the
  `const_iterator`s of CSensoryFrame, CActionCollection, CSetOfObjects,
  Viewport and CMultiMetricMap: dereferencing them now yields `X::ConstPtr`.
  Also:
  * Scene, Viewport and CSetOfObjects gain const `getByName()` overloads
  returning a ConstPtr, matching the existing getByClass() pairs.
  * CMultiMetricMap::maps is no longer a public member: use push_back(),
  size(), empty(), clearMaps(), mapByIndex(), begin()/end(), or
  mapsList() when direct manipulation of the list is really needed.
  * The mrpt_opengl renderer now keeps `const CVisualObject` handles: it
  was relying on the const-iteration hole, and every method it calls on
  the source objects was already const.
  * observationsOverlap() takes ConstPtr arguments, as it only reads.
  Porting notes added to the MRPT 3 porting guide.
* feat(mrpt_maps): retire CMetricMap::getAsSimplePointsMap()
  The virtual returned a raw pointer that was either `this` or a child map
  owned by a CMultiMetricMap, and it only ever answered for
  CSimplePointsMap: any other points map (CPointsMapXYZI,...) silently got
  a nullptr, which CPointsMap::compute3DMatchingRatio then passed straight
  into determineMatching3D().
  Replace it with a free function in mrpt_maps, `asPointsMap(map)`, which
  returns any points map, or the single child points map of a multi-metric
  map. compute3DMatchingRatio() now uses it and thus works for every points
  map type. Callers wanting ownership should use
  CMultiMetricMap::mapByClass<>() instead, which returns a real Ptr.
* fix: resolve gcc warnings flagged by the ROS buildfarm (Humble/Jazzy) (`#1412 <https://github.com/MRPT/mrpt/issues/1412>`_)
  * fix: resolve gcc warnings flagged across the ROS buildfarm (Humble/Jazzy)
  Addresses every warning category surfaced by the last Humble (Ubuntu
  Jammy, gcc-11) and Jazzy (Ubuntu Noble, gcc-13) ROS buildfarm dev jobs:
  -Wold-style-cast, -Wconversion, -Wsign-conversion, -Wfloat-conversion,
  -Wunused-parameter, -Wunused-result, -Wdeprecated-declarations,
  -Wdangling-else, -Wdangling-reference, -Wnon-virtual-dtor,
  -Woverloaded-virtual, -Wsign-compare, -Wstringop-overread, -Wshadow,
  -Wreturn-local-addr and -Wrange-loop-construct, spanning mrpt_bayes,
  mrpt_containers, mrpt_core, mrpt_graphs, mrpt_graphslam, mrpt_hwdrivers,
  mrpt_img, mrpt_maps, mrpt_math, mrpt_nav, mrpt_obs, mrpt_poses,
  mrpt_serialization, mrpt_slam, mrpt_system and mrpt_viz.
  Real bugs fixed along the way (not just silenced casts):
  * CPose3DQuat::operator[](i) const returned a temporary `double` by
  value while its `const_iterator` bound it to a `const double&`,
  making every dereference of a const iterator dangling
  (-Wreturn-local-addr).
  * CDirectedTree::Visitor and every one of its subclasses had virtual
  functions but a non-virtual destructor.
  * CSerializable's schema-archive overloads of serializeTo/serializeFrom
  were silently hidden in any class using DEFINE_SERIALIZABLE, so they
  were unreachable without explicit qualification; added the missing
  `using` declarations.
  * mrpt::Clock::time_point had no discoverable (ADL-reachable)
  operator<<, which made gtest's EXPECT_EQ/ASSERT_EQ fail to compile
  wherever it compared two timestamps on gcc-11/older libstdc++ (the
  actual Humble build error, not just a warning) -- this was the one
  case where mrpt::system::operator<<(TTimeStamp) already existed but
  lived in a namespace ADL never finds from outside mrpt::system; it is
  now `mrpt::operator<<`, with `mrpt::system` bringing it back in via a
  `using` declaration for existing qualified callers.
  * Several TKFMethod-shaped unscoped enums (TKFMethod,
  TLaserSimulUncertaintyMethod, TMatrixTextFileFormat,
  TICPCovarianceMethod) had no fixed underlying type, so a test
  casting an intentionally-invalid value into them triggered
  "conversion is unspecified" rather than the well-defined reinterpret
  a fixed underlying type gives.
  Everything else is either a widening/narrowing cast made explicit, an
  unused parameter renamed to a comment, `else`-ambiguous one-liners
  given braces, or a documented false-positive (e.g. a wrapper around a
  cached-static-map accessor gcc's -Wdangling-reference still flags)
  resolved by dropping the unnecessary named reference instead of
  suppressing the warning.
  Verified with an incremental colcon build of all 16 touched packages
  under mrpt_common's warning set (-Wall -Wextra -Wshadow -Wtype-limits
  -Wcast-align -Wparentheses -Wunused -Wpedantic -Wconversion
  -Wsign-conversion -Wdouble-promotion -Woverloaded-virtual
  -Wold-style-cast -Wnon-virtual-dtor): zero warnings left other than a
  gcc-13/-O3 -Wmaybe-uninitialized false positive on small fixed-size
  CMatrixDynamic construction that isn't present in either buildfarm log
  and isn't touched by this change. All existing unit tests
  (2000+ cases across the touched packages) still pass.
  Claude-Session: https://claude.ai/code/session_01PvVpfLXnmP5wfAzpEjd7vJ
  * fix: replace the Clock::time_point operator<< with a gtest PrintTo hook
  The operator<<(Clock::time_point) added in the previous commit, found via
  ADL because mrpt::Clock is in namespace mrpt, broke unrelated `os << x`
  calls in any translation unit with several `using namespace mrpt::...;`
  directives active at once (e.g. MonteCarloLocalization_App.cpp, which
  has ~15 of them): overload resolution for `os << particles_count`
  (a std::vector<int>) stopped finding mrpt::math's own
  operator<<(ostream&, const std::vector<T>&) entirely -- this broke all
  four CI compilers (gcc, clang, AppleClang, MSVC), all failing on the
  exact same line.
  It was also never the right fix for a second reason: mrpt::system
  already had an operator<<(TTimeStamp) doing the same thing (TTimeStamp
  being just an alias of Clock::time_point), so having both in scope at
  once was flat-out ambiguous the moment `using namespace mrpt::system;`
  and ADL both applied -- which is exactly the CObservation.cpp failure
  this was meant to fix in the first place.
  gtest's own recommended mechanism for exactly this situation --
  teaching it to print a type it doesn't otherwise know how to -- is a
  `PrintTo(value, ostream*)` overload, looked up via ADL *before* gtest
  falls back to operator<<. It doesn't add anything to the type's normal
  operator<< overload set, so it cannot participate in, or break, any
  unrelated overload resolution. mrpt::system::operator<<(TTimeStamp) is
  reverted to its original, unmodified form.
  Verified with a clean incremental build of all 21 packages (the 16 from
  the previous commit plus mrpt_libapps_gui/_cli, mrpt_gui, mrpt_apps_gui/_cli,
  which pulled in MonteCarloLocalization_App.cpp): zero errors, zero
  warnings. The four originally-affected unit test binaries
  (mrpt_core, mrpt_system, mrpt_containers, mrpt_obs) still pass in full.
  Claude-Session: https://claude.ai/code/session_01PvVpfLXnmP5wfAzpEjd7vJ
  * TKFMethod enum class defined as uint8_t
* Merge pull request `#1406 <https://github.com/MRPT/mrpt/issues/1406>`_ from MRPT/feat/coverage-math-maps-obs-slam
  Coverage pass on mrpt_math, mrpt_maps, mrpt_obs and mrpt_slam
* Fix the MSVC build: M_PI is not defined without an explicit include
  Windows CI failed with "error C2065: 'M_PI': undeclared identifier" in
  fresnel_unittest.cpp. MSVC does not define the M_PI family unless
  _USE_MATH_DEFINES is set before <cmath>; MRPT provides them from
  mrpt/core/bits_math.h (M_PI) and mrpt/core/common.h (M_PIf), which the
  other tests were picking up transitively. Both new tests that use them
  now include those headers explicitly rather than relying on that.
  Also compare getFileSize()'s uint64_t result against an unsigned literal.
* Coverage pass on mrpt_math, mrpt_maps, mrpt_obs and mrpt_slam
  Raises line coverage above the 90% goal in all four modules:
  mrpt_math 88.8% -> 95.0%, mrpt_maps 86.5% -> 90.2%,
  mrpt_obs 88.5% -> 91.6%, mrpt_slam 90.2% -> 93.5%.
  A new test helper (tests/legacy_serialization.h, in mrpt_math and
  mrpt_obs) writes an MRPT object frame with an arbitrary streaming
  version, so the backwards-compatibility branches of serializeFrom()
  can be exercised without binary fixture files.
  Bugs found and fixed along the way:
  * TLine3D::TLine3D(const TLine2D&) divided by the coefficient it had
  just tested for zero when converting a horizontal 2D line, yielding
  an infinite/NaN base point.
  * getAngleBisector(TLine2D, TLine2D)'s parallel-lines branch used the
  wrong coefficient when normalizing the second line, and never halved
  the resulting offset; the two errors cancelled only for the case the
  pre-existing test happened to use.
  * The templated intersect() over vectors of geometric objects iterated
  the inner loop up to the first vector's size, reading out of bounds,
  and its std::vector overload took the output container by value.
  * KDTreeCapable's radius-limited k-NN searches left the coordinate
  output vectors sized at knn while trimming the index/distance ones,
  handing back stale entries.
  * CHistogram::createWithFixedWidth() was not static, so the documented
  usage (and its only caller, CMonteCarlo) did not compile.
  * KLD_Gaussians() called a non-existent inverse_LLt() overload and the
  row-vector form of the quadratic-form helper; it was uninstantiable.
  * CGasConcentrationGridMap2D::simulateAdvection() read past the end of
  an empty matrix for any map type other than mrKalmanApproximate; it
  now returns false with an error log.
* Merge branch 'develop' into fix/stereo-rectify-map-axis-swap
* Contributors: Jose Luis Blanco-Claraco

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

