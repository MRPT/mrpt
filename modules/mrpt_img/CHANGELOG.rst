^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_img
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.1.4 (2026-09-04)
------------------
* fix(mrpt_img): fix CImage grayscale deserialization of legacy rawlogs storing PixelDepth as 0 (`#1399 <https://github.com/MRPT/mrpt/issues/1399>`_).
* Contributors: Jose Luis Blanco-Claraco

3.1.3 (2026-08-12)
------------------
* fix(mrpt_img): add missing <algorithm> include for std::min/max initializer-list overloads.
* mrpt_img: increase code coverage and fix bugs.
  Add tests across CImage, drawing primitives, and camera classes. Fix bugs in CMappedImage interpolation, RGB-to-HSV conversion, undistort_points, FFT cross-correlation, KLT bounds, and JPEG stream saving.
* Contributors: Jose Luis Blanco-Claraco

3.1.2 (2026-07-07)
------------------

3.1.1 (2026-07-04)
------------------

3.1.0 (2026-07-03)
------------------
* chore: remove undesired bin file
* Contributors: Jose Luis Blanco-Claraco

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

