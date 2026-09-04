^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.1.4 (2026-09-04)
------------------
* fix(mrpt_core): fix precision loss in Clock::toDouble() (up to ~1 us) by doing the epoch shift in the integer domain instead of floating point.
* test(mrpt_core): add regression test for Clock::toDouble unsigned wraparound on pre-epoch timestamps (`#1386 <https://github.com/MRPT/mrpt/issues/1386>`_).
* Contributors: Jose Luis Blanco-Claraco

3.1.3 (2026-08-12)
------------------

3.1.2 (2026-07-07)
------------------

3.1.1 (2026-07-04)
------------------

3.1.0 (2026-07-03)
------------------
* chore: ASSERT\_ macros better logging
* Contributors: Jose Luis Blanco-Claraco

3.0.4 (2026-06-17)
------------------

3.0.3 (2026-06-15)
------------------

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

