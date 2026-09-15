^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_nav
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
* Merge pull request `#1410 <https://github.com/MRPT/mrpt/issues/1410>`_ from MRPT/feat/nav-api-modernization
  mrpt_nav: modernize the PTG/reactive API and fix 7 TP-Space bugs
* fix(mrpt_nav): address review feedback on the TP-Space pass
  * calc_move_candidate_scores() passed the *normalized* collision-free
  distance to getPathStepForDist(), which takes pseudometers -- the ETA
  factor 300 lines below does `d * ref_dist` for the very same call. The
  "end of trajectory" pose behind robpose\_*, dist_eucl_final and the target
  slow-down check was read `ref_distance` times too early along the path:
  0.74 m instead of 4 m in the new regression test. Switched to
  getPathStepForDistClamped() as well, since with the units fixed a
  candidate at the very end of the path could now trip the assertion.
  * CPTG_Holo_Blend::m_pathStepCountCache counts path *time steps*, so every
  write to m_pathTimeStep now goes through setPathTimeStep(), which clears
  the cache. It also rejects non-finite and non-positive values, including
  ones read from a stream, which getPathStepForDist() would otherwise
  divide by before casting to an unsigned step index.
  252/252 tests pass.
  Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_019sunyEVmTCDcr4vjjWKQQc
* refactor(mrpt_nav): modernize the PTG/reactive API and fix 7 TP-Space bugs
  API: replace bool+out-param signatures with std::optional, matching what
  inverseMap_WS2TP() already did. Old forms stay as deprecated shims:
  * CParameterizedTrajectoryGenerator::getPathStepForDist(k, dist). The old
  3-arg form also wrote the last path step into out_step when returning
  false, and two call sites quietly depended on that; the explicit
  getPathStepForDistClamped() now covers it.
  * nav_plan_geometry_utils::collision_free_dist\_{segment,arc}_circ_robot()
  * PlannerSimple2D::computePath()
  ClearanceDiagram::getClearance()'s bool flag becomes enum class
  ClearanceQuery. CPTG_Holo_Blend's mutable global PATH_TIME_STEP becomes the
  per-instance `path_time_step` config key plus setPathTimeStep(), and `eps`
  becomes a constexpr EPSILON (no shims possible for either).
  setScorePriorty() gains a correctly spelled setScorePriority(). Deleted:
  updateClearancePost(), a documented no-op since 2017, and
  CAbstractHolonomicReactiveMethod::Create(), declared in a public header but
  never defined anywhere, so any call was a link error.
  Bugs found and fixed, each with a regression test that fails without it:
  1. getClearance()'s two modes were swapped relative to its own docs, to
  every call site's comment and to the ptg-configurator's UI label, so the
  reactive navigator's `clearance` and `clearance_path` score factors held
  each other's values.
  2. initClearanceDiagram() keyed samples by the raw path distance in meters,
  while every consumer treats those keys as normalized [0,1] TPS distances.
  3. It also sampled steps 0, incr, 2*incr... whereas
  evalClearanceSingleObstacle() evaluates incr, 2*incr..., so each
  clearance was filed under a shorter distance than it was measured at.
  4. CHolonomicVFF::navigate() assigned desiredSpeed inside the
  `if (m_enableApproachTargetSlowDown)` block, so with the slow-down
  disabled it returned zero speed and the robot never moved.
  5. CHolonomicFullEval slowed down for targets.front(); NavInput documents
  the last target as the highest-priority one.
  6. CPTG_Holo_Blend used V_MAX as the post-ramp cruise speed in
  getPathDist(), getPathStepForDist() and updateTPObstacleSingle(), while
  getPathPose() advances at the direction-dependent internal_get_v(dir):
  with an expr_V set, poses and distances disagreed. inverseMap_WS2TP()
  likewise pinned T_ramp to T_ramp_max instead of the per-direction value.
  7. collision_free_dist_arc_circ_robot()'s closed form divided by the
  obstacle's x, returning NaN for any obstacle on the turn-center axis.
  Rewritten as a two-circle intersection, agreeing with the old formula to
  1.7e-11 over 21k random collision cases, and now returning 0 when the
  robot starts already in collision rather than the exit distance.
  Math model: the arc-length quadrature of CPTG_Holo_Blend moves from a
  15-interval trapezoidal rule to 16-interval Simpson, the same number of
  function evaluations for ~25x lower mean relative error, plus an exact
  branch for the degenerate case where the integrand is sqrt(a)*|t-r|.
  Docs: rewrite the PTG base class description around what TP-Space is and
  drop its 20-year change log; fix the stale out-params left in
  inverseMap_WS2TP()'s docs. New headless example nav_ptg_tpspace walks the
  whole WS -> TP-Space -> velocity-command round trip.
  251/251 tests pass (was 246).
  Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_019sunyEVmTCDcr4vjjWKQQc
* Merge branch 'develop' into fix/stereo-rectify-map-axis-swap
* Contributors: Jose Luis Blanco-Claraco

3.1.4 (2026-09-04)
------------------
* Merge pull request `#1388 <https://github.com/MRPT/mrpt/issues/1388>`_ from MRPT/test/coverage-nav-kinematics
  test(mrpt_nav, mrpt_kinematics): raise coverage to 90%/97% and fix 10 bugs found on the way
* test(mrpt_nav): make two timing-sensitive navigator tests platform-independent
  The MSVC CI job failed on not_approaching_the_target_times_out: the test
  set alarm_seems_not_approaching_target_timeout to 0 and relied on two
  consecutive Clock::now() calls differing, which the Windows clock
  resolution does not guarantee. Both this test and the waypoint-alignment
  one now drive a simulated clock instead of depending on how fast the loop
  happens to run.
* fix(mrpt_nav): drop all cached expression state when a formula fails to compile
  Addresses review feedback: clearing only the map that failed left score
  names already registered as variables behind, so a first score that
  compiles followed by one that does not made the *next* decide() throw
  "Expression name already exists as an input variable". Both failure
  paths now go through clear(), and a regression test covers the case.
  Also replaces the hard-coded /tmp PTG cache directory in the new planner
  test with a portable temporary path.
* test(mrpt_nav): simulator robot interfaces, manual-sequence navigator and 3D reactive nav
  Covers the last sizeable gaps in the module: the ready-made
  CRobot2NavInterfaceForSimulator\_{Holo,DiffDriven} adapters, the
  pre-programmed velocity-sequence navigator (config parsing for both
  kinematic models, the malformed-input rejections and the
  failed-command emergency stop), a CReactiveNavigationSystem3D run from
  an in-memory config, and the PTG collision-grid cache-file save/reload
  path.
  Also applies clang-format-14 to the new test files.
  Two more bugs found and fixed:
  * CReactiveNavigationSystem3D::saveConfigFile() never called the
  CAbstractPTGBasedReactive implementation, unlike its 2D sibling and
  contrary to the documented contract, so saving a 3D navigator's
  configuration produced a stub with only HEIGHT_LEVELS and PTG_COUNT
  that could not be loaded back.
  * PlannerSimple2D::computePath()'s "are the endpoints inside the grid?"
  guard read `!(originInside || !targetInside)`, which flags exactly the
  wrong case: it only reported notFound when the origin was outside *and*
  the target inside, and let an out-of-grid target (or both endpoints out
  of grid) fall through into the search, contrary to its own documented
  behavior.
* test(mrpt_nav): holonomic method configuration and reactive-navigation variants
  Round-trips the config files and serialization of CHolonomicVFF/ND/FullEval
  and their log records, and adds reactive-navigation runs driven by fully
  in-memory configurations (so they never silently skip when the shared
  config files are missing) covering the optional features of
  CAbstractPTGBasedReactive: the delays model, clearance evaluation,
  velocity filtering, disabled obstacle filtering, log-record keeping and
  file writing, PTG restriction, runtime holonomic-method switching,
  robot-shape changes and the obstacle-sensor failure path.
  The runs use a CPTG_Holo_Blend so the "NOP cmdvel" PTG-continuation
  branches are exercised too, which the differential-drive PTGs of the
  existing rnav tests cannot reach.
* test(mrpt_nav): cover the clearance diagram, planners, log records and motion optimizers
  Adds unit tests for the pieces of mrpt_nav that no existing test
  touched: ClearanceDiagram (index mapping, serialization, 3D rendering),
  the arc-vs-circular-robot collision helper, the default
  CRobot2NavInterface callbacks, NavigationLogger, VelocityFilter, the
  multi-objective motion optimizers, the RRT move-tree 3D renderer and a
  fully-populated CLogFileRecord round-trip.
  Bugs found and fixed along the way:
  * CMultiObjectiveMotionOptimizerBase::decide() returned -1 from a
  function returning std::optional<size_t> when a user formula failed to
  compile. That is an *engaged* optional holding SIZE_MAX, so instead of
  the documented "no valid candidate", callers indexed the candidate
  vector out of bounds.
  * CMultiObjectiveMotionOptimizerBase::clear() dropped the compiled score
  expressions but kept the variable table, so the next decide() threw
  "Expression name already exists as an input variable" -- i.e. the
  documented way to re-apply changed parameters was unusable.
  * CLogFileRecord's legacy (pre-v15) deserialization wrote every velocity
  command component into the wrong slot: one loop indexed with the outer
  loop variable, and the oldest format wrote both v and w into element 0.
* test(mrpt_kinematics,mrpt_nav): unit tests for the kinematics and navigation stack
  mrpt_kinematics had no C++ unit tests at all; mrpt_nav's were limited to
  the reactive-navigation integration tests plus a few data structures.
  New tests cover CVehicleVelCmd (both kinematic models), the vehicle
  simulators, CKinematicChain, TWaypoint/TWaypointSequence, the whole PTG
  family (including CPTG_DiffDrive_CC/CS/CCS, previously never
  instantiated) and the CAbstractNavigator/CWaypointsNavigator state
  machine.
  Bugs found and fixed along the way:
  * CVehicleVelCmd's copy constructor delegated to operator=(), which
  dispatches pure virtual methods while the derived object is still under
  construction: copy-constructing any velocity command aborted the
  process.
  * CAbstractNavigator::internal_onStartNewNavigation() cleared the cached
  pose history but left the last-query timestamp untouched, so the
  updateCurrentPoseAndSpeeds() call right after it could be skipped by
  its minimum-period throttle and leave the cache empty. The following
  ASSERT\_(!m_latestPoses.empty()) then threw on the first navigation step
  of every waypoint mission and of every relative-target navigation.
  * CAbstractNavigator::performNavigationStepNavigating() restored
  m_navigationState from its entry value on exit, undoing the very
  transitions it had just decided: neither an exception nor
  doEmergencyStop() could leave the navigator in NAV_ERROR. The
  assignment was meant for m_lastNavigationState, mirroring
  navigationStep().
  * CWaypointsNavigator::checkHasReachedTarget() dereferenced a possibly
  null waypoint pointer whenever the robot was farther than the allowed
  distance and the waypoint list was empty or its index out of range.
  * CParameterizedTrajectoryGenerator::Alpha2index() discarded the result
  of wrapToPi(), so directions outside [-pi,pi] were clamped to the first
  or last path instead of wrapping around.
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

