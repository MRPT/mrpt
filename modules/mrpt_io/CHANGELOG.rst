^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_io
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

3.1.1 (2026-07-04)
------------------

3.1.0 (2026-07-03)
------------------

3.0.4 (2026-06-17)
------------------

3.0.3 (2026-06-15)
------------------
* Increase unit test coverage for mrpt_math, mrpt_io, mrpt_system (`#1365 <https://github.com/MRPT/mrpt/issues/1365>`_)
  * Increase unit test coverage for mrpt_math, mrpt_io, mrpt_system
  Add unit tests for TTwist3D, TObject2D/TObject3D (serialization and
  2D/3D casting), RANSAC line fitting, CFileStream, CPipe, and
  CDirectoryExplorer.
  Fix a bug in TObject2D::operator<< and TObject3D::operator<< where
  serialization always threw "Unexpected type index" regardless of the
  actual variant content, due to an unconditional THROW_EXCEPTION after
  the if/else-if chain instead of in an else branch.
  * Apply clang-format-14 to new test files
  * Fix MSVC build: M_PI_2 is not a standard macro
  Use M_PI / 2 instead, matching the M_PI definition already provided
  by mrpt_core's bits_math.h for all platforms.
  * Fix CDirectoryExplorer::explore() on Windows to skip "." and ".." entries
  The Windows (FindFirstFile/FindNextFile) implementation listed "." and
  ".." as regular directory entries, unlike the POSIX implementation
  which already skips them. This caused the new
  CDirectoryExplorer_unittest.cpp tests to fail on Windows CI.
* Contributors: Jose Luis Blanco-Claraco

3.0.2 (2026-06-11)
------------------

3.0.1 (2026-06-11)
------------------

3.0.0 (2026-06-06)
------------------

2.20.0 (2026-06-06)
-------------------
* Last release of the 2.x series. Starting from 3.0.0, changes are tracked
  in each module's own CHANGELOG.rst file.
* Contributors: Jose Luis Blanco-Claraco

