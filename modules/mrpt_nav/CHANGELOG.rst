^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_nav
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

