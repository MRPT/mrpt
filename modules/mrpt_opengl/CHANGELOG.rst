^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_opengl
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.4.0 (2026-10-04)
------------------
* Python: commit generated .pyi type stubs for all modules
* Render pipeline: fix scene sync and rendering bugs, add frustum culling
* Shadows: casters far towards the light are no longer clipped; honor the viewport shadow map size
* Group Assimp textured meshes by alpha mode; validate deserialized alpha mode
* mrpt_viz, mrpt_opengl: alpha cutout for textures (TAlphaMode)
* mrpt_opengl: keep vertex colors on textured triangles with shadows
* mrpt_opengl: always restore the previous cull face mode
* mrpt_opengl: culled faces still cast shadows
* mrpt_opengl: normal maps on mirrored UV mappings
* mrpt_opengl: fix normal maps on lit textured triangles
* mrpt_opengl: bound the loops that clear pending GL errors
* Fix test failures on ARM and s390x Ubuntu PPA builds
* Contributors: Jose Luis Blanco-Claraco

3.3.1 (2026-09-29)
------------------
* viz, opengl: fix PLY import, bounding boxes, sky boxes, viewport modes and SSAO; add tests.
* mrpt_opengl: reference count textures shared by several users (#1433).
* Contributors: Jose Luis Blanco-Claraco

3.3.0 (2026-09-26)
------------------
* Python bindings: datasets, maps, localization, sensors (+ fixes) (`#1429 <https://github.com/MRPT/mrpt/issues/1429>`_)
* fix(viz): stale pose of containers updated by assignment
* changelogs
* mrpt_opengl: use renamed libgles-dev rosdep key
* mrpt_opengl: depend on new opengl-es rosdep key
* Contributors: Jose Luis Blanco-Claraco


3.2.0 (2026-09-16)
------------------
* fix: restore 2D overlay rendering for scene cameras and text labels.
* refactor: tighten pointer const-correctness and API consistency across the render stack.
* fix: restore missing render buffers and fix rendering regressions uncovered by the new tests.
* Contributors: Jose Luis Blanco-Claraco

3.1.4 (2026-09-04)
------------------

3.1.3 (2026-08-12)
------------------
* test(mrpt_viz): add extensive unit test coverage for mrpt::viz classes.
  Fix bugs in CPolyhedron init, PLY importer, and CMesh3D triangle face-normal computation.
* test(mrpt_viz, mrpt_opengl): add framebuffer regression tests for all drawing primitives.
* Contributors: Jose Luis Blanco-Claraco

3.1.2 (2026-07-07)
------------------

3.1.1 (2026-07-04)
------------------

3.1.0 (2026-07-03)
------------------

3.0.4 (2026-06-17)
------------------
* fix: conservative use depend to ensure opengl binary libs are added downstream of mrpt_opengl
* Merge pull request `#1371 <https://github.com/MRPT/mrpt/issues/1371>`_ from wentasah/export-opengl
  mrpt_opengl: Add opengl build_depend back
* mrpt_opengl: Add opengl build_depend back
  In a recent commit, build_depend was replaced with
  build_export_depend. But it seems that both build_depend and
  build_export_depend need to be specified. Without build_depend, ROS
  build farm complains about OpenGL not being available:
  Could NOT find OpenGL (missing: OPENGL_opengl_LIBRARY OPENGL_glx_LIBRARY OPENGL_INCLUDE_DIR)
* Contributors: Jose Luis Blanco-Claraco, Michal Sojka

3.0.3 (2026-06-15)
------------------
* Merge pull request `#1368 <https://github.com/MRPT/mrpt/issues/1368>`_ from wentasah/export-opengl
  mrpt_opengl: Export opengl dependency
* Contributors: Jose Luis Blanco-Claraco, Michal Sojka

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

