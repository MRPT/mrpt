^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_kinematics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.1.4 (2026-09-04)
------------------
* test(mrpt_kinematics): add a unit test suite (module had none), raising coverage to 97% (`#1388 <https://github.com/MRPT/mrpt/issues/1388>`_).
* fix(mrpt_kinematics): CVehicleVelCmd's copy constructor delegated to operator=(), dispatching pure virtual methods during construction and aborting the process.
* docs(mrpt_kinematics): CKinematicChain::recomputeAllPoses()'s documented `pose0` argument is actually ignored in favor of setOriginPose(); doc corrected to match.
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

3.0.0 (2026-06-06)
------------------

2.20.0 (2026-06-06)
-------------------
* Last release of the 2.x series. Starting from 3.0.0, changes are tracked
  in each module's own CHANGELOG.rst file.
* Contributors: Jose Luis Blanco-Claraco

