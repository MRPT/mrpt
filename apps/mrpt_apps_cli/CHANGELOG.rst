^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_apps_cli
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge pull request `#1413 <https://github.com/MRPT/mrpt/issues/1413>`_ from MRPT/feat/api-cleanups-3.1
  feat: API cleanups before the next minor bump (archives, const-correctness, points-map accessor)
* feat(mrpt_obs,mrpt_viz,mrpt_maps): deep const-correctness in smart-pointer containers
  Reading through a `const` container of `X::Ptr` handed out mutable
  pointees, so constness stopped at the container. Add a small
  `mrpt::containers::deep_const_iterator` proxy and use it for the
  `const_iterator`s of CSensoryFrame, CActionCollection, CSetOfObjects,
  Viewport and CMultiMetricMap: dereferencing them now yields `X::ConstPtr`.
  Also:
  * Scene, Viewport and CSetOfObjects gain const `getByName()` overloads
  returning a ConstPtr, matching the existing getByClass() pairs.
  * CMultiMetricMap::maps is no longer a public member: use push_back(),
  size(), empty(), clearMaps(), mapByIndex(), begin()/end(), or
  mapsList() when direct manipulation of the list is really needed.
  * The mrpt_opengl renderer now keeps `const CVisualObject` handles: it
  was relying on the const-iteration hole, and every method it calls on
  the source objects was already const.
  * observationsOverlap() takes ConstPtr arguments, as it only reads.
  Porting notes added to the MRPT 3 porting guide.
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

