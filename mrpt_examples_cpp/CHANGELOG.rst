^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_examples_cpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

