^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_gui
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge pull request `#1405 <https://github.com/MRPT/mrpt/issues/1405>`_ from MRPT/feat/coverage-graphslam-gui-xvfb
  Coverage pass on mrpt_graphslam and mrpt_gui (headless GUI tests under Xvfb)
* Address PR review: CTicTac alignment on MSVC, read-only CI token
  * CTicTac's storage is now uint64_t[4] rather than unsigned long[4]. The
  latter is 4 bytes and 4-byte aligned on MSVC, so dropping the (excessive,
  crash-causing) alignas(16) left it under-aligned for the LARGE_INTEGER it
  is reinterpreted as; uint64_t is 8-byte aligned everywhere, which is what
  both LARGE_INTEGER and struct timespec need, and is still below the
  over-alignment that broke virtual-base layout. The constructor now asserts
  alignment as well as size, per platform.
  * build-linux.yml declares top-level `permissions: contents: read`; nothing
  in the workflow writes to the repository.
  Also documents, in the GUI test helper, that a configured-but-unreachable
  DISPLAY is deliberately left to fail rather than skip, and how to opt out.
  Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01Uitwp7fw5JbCYkiKe7QYnh
* Report the actual reason when the GUI tests skip
  The macOS and Windows CI jobs build with -DDISABLE_WXWIDGETS=ON, so the
  window tests skip there for that reason, not for a missing display; the
  message told the reader to set DISPLAY or use xvfb-run, which does not
  apply. The skip reason is now computed once and reported verbatim, covering
  all three cases (no wxWidgets, no window server, MRPT_SKIP_GUI_TESTS=1).
  Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01Uitwp7fw5JbCYkiKe7QYnh
* Coverage pass on mrpt_graphslam and mrpt_gui; fix the bugs it uncovered
  mrpt_graphslam 73.3% -> 81.6%, mrpt_gui 0.5% -> 40.6% lines. The mrpt_gui
  window classes are now tested headlessly under a virtual X display, so CI
  installs xvfb + Mesa's software GL and wraps `colcon test` in `xvfb-run`;
  the tests GTEST_SKIP() when no display is reachable.
  Bugs found and fixed along the way:
  * CTicTac declared its timestamp storage alignas(16) for no reason. That
  alignment propagated through CTimeLogger up to
  CRegistrationDeciderOrOptimizer, which is used as a virtual base, and GCC
  then emitted movdqa on a subobject that is only 8-byte aligned inside a
  derived class: constructing a CFixedIntervalsNRD segfaulted in every
  optimized build. Same failure mode already documented in CMatrixFixed.
  * CNodeRegistrationDecider::registerNewNodeAtEnd() placed the root node at
  the current pose estimate instead of the origin, applying the accumulated
  motion twice.
  * The three decider TParams structs left registration_max_distance/_angle
  uninitialized, making their documented defaults dead code.
  * CIncrementalNodeRegistrationDecider and
  TUncertaintyPath::hasLowerUncertaintyThan() did not compile if
  instantiated (undefined report_sep, unqualified INVALID_NODEID, a const
  method calling a non-const one).
  * CGlCanvasBase's mouse setters and CGlCanvasBaseHeadless::renderError()
  were declared but never defined. updateLastPos() is restored and wired
  into the wx mouse handlers, which never recorded the pointer position, so
  CDisplayWindow3D::getLastMousePosition() and 3D picking always returned
  pixel (0,0). The two click setters were dead and are removed.
  * wxImage2MRPTImage() swapped red and blue: a leftover from MRPT 2.x, when
  CImage stored BGR.
  * CWindowObserver compared key modifiers with == against a magic 8192, so
  Ctrl+C was missed if any other modifier was held.
  * CNodeRegistrationDecider_impl.h had `using namespace std;` at global scope
  in a public header.
  Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01Uitwp7fw5JbCYkiKe7QYnh
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
* fix: enable isolated/Debian-package builds of apps_gui
  Two issues surface when building packages in isolation (as the ROS build farm
  and .deb packaging do), where sibling modules are only available as installed
  packages, not as source:
  - mrpt-performance/perf-graphslam.cpp reached into another module's test
  sources via a relative path (../../../modules/mrpt_graphslam/tests/...).
  Vendor a local copy of the small test helper header instead.
  - mrpt_gui installs the public header nanogui/opengl.h which includes
  <GLFW/glfw3.h>, but only declared libglfw3-dev as a build dependency. Make it
  a public <depend> so downstream consumers get the GLFW headers.
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

