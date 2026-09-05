^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_graphslam
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.1.4 (2026-09-04)
------------------
* test(mrpt_graphslam): raise unit test coverage of TSlidingWindow and CEdgeCounter, previously untested (41%->72% lines) (`#1389 <https://github.com/MRPT/mrpt/issues/1389>`_).
* fix(mrpt_graphslam): TSlidingWindow::getMean()/getStdDev() gave NaN/wrong results on a partially-filled window; resizeWindow() left the std-dev cache stale.
* fix(mrpt_graphslam): CEdgeCounter::clearAllEdges() left m_unique_edges stale.
* Contributors: Jose Luis Blanco-Claraco

3.1.3 (2026-08-12)
------------------
* fix(mrpt_graphslam): match fully-qualified class names for constraint-type whitelist. Skip RGBD-TUM info parsing when rawlog is empty.
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

