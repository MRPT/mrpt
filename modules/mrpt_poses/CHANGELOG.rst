^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_poses
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.6.3 (2026-11-13)
------------------
* CPoseInterpolatorBase: fix the documented default interpolation method (`#1503 <https://github.com/MRPT/mrpt/issues/1503>`_)
* Python bindings: Gaussian PDF composition, copies, inverses, CPosePDFSOG and interpolation method (`#1502 <https://github.com/MRPT/mrpt/issues/1502>`_)
* FrameTransformer: fix -Wshadow warning in lookupTransform()
* Contributors: Jose Luis Blanco-Claraco

3.6.2 (2026-11-07)
------------------
* CPose3DQuatPDFGaussianInf::evaluatePDF(): density of degenerate Gaussians (`#1500 <https://github.com/MRPT/mrpt/issues/1500>`_)
* Python wheel: nanogui 3D windows from Python, complete C++ SDK, NumPy arrays as MRPT matrices (`#1496 <https://github.com/MRPT/mrpt/issues/1496>`_)
* inverse_LLt() throws for matrices that are not positive-definite; pseudo-inverse for quaternion information PDFs (`#1494 <https://github.com/MRPT/mrpt/issues/1494>`_)
* Math review: fixes and tests for transformations, probability distributions and their users (`#1492 <https://github.com/MRPT/mrpt/issues/1492>`_)
* mrpt_poses: to_mrpt_se3_pdf() throws at the pitch = +-90 deg singularity (`#1491 <https://github.com/MRPT/mrpt/issues/1491>`_)
* Fix to_gtsam_se3_cov6() covariance conversion; add its exact inverse to_mrpt_se3_pdf() (`#1490 <https://github.com/MRPT/mrpt/issues/1490>`_)
* Text matrix and TUM loaders: indented comments, blank lines, non-finite rows (`#1486 <https://github.com/MRPT/mrpt/issues/1486>`_)
* Contributors: Jose Luis Blanco-Claraco

3.6.1 (2026-11-01)
------------------

3.6.0 (2026-10-28)
------------------

3.5.3 (2026-10-16)
------------------

3.5.2 (2026-10-11)
------------------

3.5.1 (2026-10-09)
------------------
* Fix wrong CQuaternion::jacobian_rodrigues_from_quat() (`#1460 <https://github.com/MRPT/mrpt/issues/1460>`_)
* Contributors: Jose Luis Blanco-Claraco

3.5.0 (2026-10-07)
------------------
* docs: fix nearly all Sphinx warnings
* Python bindings: docstrings for all members, regenerated .pyi stubs
* Contributors: Jose Luis Blanco-Claraco

3.4.0 (2026-10-04)
------------------
* Python: commit generated .pyi type stubs for all modules
* Contributors: Jose Luis Blanco-Claraco

3.3.1 (2026-09-29)
------------------

3.3.0 (2026-09-26)
------------------
* fix: qualify mrpt::format() calls to avoid ambiguity with std::format
* Python bindings: datasets, maps, localization, sensors (+ fixes) (`#1429 <https://github.com/MRPT/mrpt/issues/1429>`_)
* changelogs
* Contributors: Jose Luis Blanco-Claraco

3.2.0 (2026-09-16)
------------------
* refactor: align API and const-correctness updates with the broader 3.x cleanup pass.
* fix: resolve ROS buildfarm warning issues and fix the underlying correctness bugs they surfaced.
* fix: address stereo-rectification and pose-related regressions in the latest 3.x fixes.
* Contributors: Jose Luis Blanco-Claraco

3.1.4 (2026-09-04)
------------------

3.1.3 (2026-08-12)
------------------

3.1.2 (2026-07-07)
------------------

3.1.1 (2026-07-04)
------------------
* add more mrpt_poses unit tests
* Contributors: Jose Luis Blanco-Claraco

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

