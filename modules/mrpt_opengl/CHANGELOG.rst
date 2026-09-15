^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_opengl
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge pull request `#1414 <https://github.com/MRPT/mrpt/issues/1414>`_ from MRPT/fix/2d-overlay-rendering
  Fix 2D overlay rendering: scene cameras and CText labels
* fix(mrpt_opengl): restore 2D overlay rendering (scene cameras and CText)
  Two defects in the new rendering pipeline broke any scene drawing a 2D
  overlay, RawLogViewer's bottom timeline being the visible case: it showed
  up in perspective, with oversized labels, instead of a flat 2D plot.
  * CompiledViewport::updateFromVizViewport() took the viewport's own camera
  instead of the one actually used for rendering, so a CCamera inserted as
  a scene object was ignored and its setNoProjection() never took effect.
  Viewport::resolveActiveCamera() now exposes the resolution logic that
  was already in place for ray casting.
  * Text2DLabelProxy rendered CText as plain world geometry, ignoring the
  font height altogether: a label meant to be N pixels tall was drawn N
  world units tall. It is now placed at the projection of its 3D origin
  and scaled to the requested size in pixels, as CText is defined to
  behave. This also restores enableShowName() labels.
  Reference images: linePointPrimitives had the rotated, oversized "MRPT"
  label baked in, and the two camera ones were missing the box name label.
  Regenerated. Note their tolerance (a 5000.0 sum of absolute differences
  over the whole frame) let an 870-point change through unnoticed, which is
  why the new tests assert geometric invariants on the pixels instead:
  extents that must not taper, and glyph sizes that must not depend on the
  camera distance.
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
* Merge pull request `#1407 <https://github.com/MRPT/mrpt/issues/1407>`_ from MRPT/feat/coverage-viz-opengl
  Coverage pass on mrpt_viz: 70% -> 87%, fixing four classes that rendered nothing
* Fix a NaN from TLine3D::distance(), and address review feedback
  macOS CI caught the first one: TLine3D::distance() computes
  sqrt(d2 - dv*dv/v2), whose radicand is zero in exact arithmetic for a point
  lying on the line but can round slightly negative, giving a NaN. Clamped at
  zero, with a regression test covering both axis-aligned and oblique lines.
  From the PR review:
  * CVectorField3D::serializeFrom() left its v2-only fields untouched when
  reading a v1 stream, so loading into a reused object kept the previous
  color-mapping settings. They are reset to the constructor defaults now.
  * The PLY importer kept a single color scale for all three channels, which
  decodes the wrong range for a file mixing e.g. "uchar red" with
  "float green". One scale per channel, with a test for that layout.
  * CSimpleLine::setLineCoords(TPoint3Df, TPoint3Df) did not call
  notifyChange(), unlike its scalar overload, so it could leave a stale line
  buffer active now that the class has an updateBuffers().
  * CFrustum emitted 4 of its 12 edges twice, a leftover of the GL_LINE_STRIP
  path it was ported from.
  * CDisk accepted a slice count below 3 and only failed on the assertion
  inside updateBuffers(); the constructor and setSlicesCount() reject it now.
  The linePointPrimitives reference image is refreshed once more: restoring
  CPointCloud::updateBuffers() means its point cloud finally honors
  setColor(), so it renders white as the test asks instead of picking up a
  stale vertex attribute.
  Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_014Mjmvqb883MdKyobSADXvJ
* viz, opengl: apply clang-format
  Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_014Mjmvqb883MdKyobSADXvJ
* viz: restore the missing render buffers of CSetOfLines, CSimpleLine, CDisk and CFrustum
  These four classes never overrode updateBuffers(), so their geometry never
  reached the CPU-side vertex buffers the renderer uploads to the GPU: they
  were simply invisible in any scene. The implementations are back, ported
  from the equivalent MRPT 2.x shader callbacks (dropped along the way in the
  viz/opengl split), with CFrustum honoring its draw-lines/draw-planes flags
  and CSetOfLines only emitting vertex dots when a non-zero point size was
  requested.
  New RenderBuffers_unittest.cpp asserts that every visual object fills the
  buffer it is supposed to fill. Unlike the offscreen-rendering tests, this
  needs no GL context, so it runs everywhere.
  The offscreen-rendering tests were passing on the broken output because
  their reference images had been captured with the defect present; both
  affected images are regenerated here. Their duplicated imageDiff() helper
  now lives in a shared tests/render_reference.h, which also adds an
  MRPT_UPDATE_RENDER_REFERENCES=1 environment switch to rewrite a reference
  image after an intended visual change.
  Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_014Mjmvqb883MdKyobSADXvJ
* Merge branch 'develop' into fix/stereo-rectify-map-axis-swap
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

