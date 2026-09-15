^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_examples_cpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge pull request `#1410 <https://github.com/MRPT/mrpt/issues/1410>`_ from MRPT/feat/nav-api-modernization
  mrpt_nav: modernize the PTG/reactive API and fix 7 TP-Space bugs
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
* fix: build errors from limiting Eigen3 visibility + KF Joseph form
  Fix CI build failures introduced by making Eigen3 a private build
  dependency:
  - 2d-slam-demo: drop unnecessary Eigen::aligned_allocator from
  m_historicData; a plain std::vector suffices and no longer needs
  Eigen headers transitively.
  - topography_gps_coords_example: this example genuinely uses MRPT
  matrix methods that require <Eigen/Dense> in the calling TU, so
  link Eigen3::Eigen explicitly instead of relying on transitive
  exposure.
  Also fix a correctness bug in the new dense Joseph-form covariance
  update: it added K*S*K^T (S = H*P*H^T + R), double-counting the
  H*P*H^T*K^T term already present in (I-K*H)*P*(I-K*H)^T. Use the
  measurement noise K*R*K^T instead, matching the algebraically
  correct sparse update path.
* refactor: limit visibility of eigen3 as build dep
* Contributors: Jose Luis Blanco-Claraco

3.0.0 (2026-06-06)
------------------

2.20.0 (2026-06-06)
-------------------
* Last release of the 2.x series. Starting from 3.0.0, changes are tracked
  in each module's own CHANGELOG.rst file.
* Contributors: Jose Luis Blanco-Claraco

