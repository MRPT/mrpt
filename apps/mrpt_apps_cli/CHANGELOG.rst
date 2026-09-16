^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_apps_cli
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.2.0 (2026-09-16)
------------------
* feat: API cleanup pass for archive handling, const-correctness, and points-map accessors before the next minor bump.
* feat(mrpt_obs,mrpt_viz,mrpt_maps): enforce deep const-correctness in smart-pointer containers and return ConstPtr from const iteration paths.
* feat: add const-aware accessors and update container APIs to avoid exposing mutable pointees through const containers.
* feat: keep renderer and overlap helpers const-safe, and document the migration notes in the MRPT 3 porting guide.
* Merge branch 'develop' into fix/stereo-rectify-map-axis-swap
* Contributors: Jose Luis Blanco-Claraco

3.1.4 (2026-09-04)
------------------
* fix(mrpt_apps_cli): add missing <iostream> include in carmen2rawlog, gps2rawlog and carmen2simplemap, fixing the MSVC build.
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

