^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_viz
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge pull request `#1414 <https://github.com/MRPT/mrpt/issues/1414>`_ from MRPT/fix/2d-overlay-rendering
  Fix 2D overlay rendering: scene cameras and CText labels
* docs(mrpt_viz): describe what CText's font size actually means on screen
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
* fix: resolve gcc warnings flagged by the ROS buildfarm (Humble/Jazzy) (`#1412 <https://github.com/MRPT/mrpt/issues/1412>`_)
  * fix: resolve gcc warnings flagged across the ROS buildfarm (Humble/Jazzy)
  Addresses every warning category surfaced by the last Humble (Ubuntu
  Jammy, gcc-11) and Jazzy (Ubuntu Noble, gcc-13) ROS buildfarm dev jobs:
  -Wold-style-cast, -Wconversion, -Wsign-conversion, -Wfloat-conversion,
  -Wunused-parameter, -Wunused-result, -Wdeprecated-declarations,
  -Wdangling-else, -Wdangling-reference, -Wnon-virtual-dtor,
  -Woverloaded-virtual, -Wsign-compare, -Wstringop-overread, -Wshadow,
  -Wreturn-local-addr and -Wrange-loop-construct, spanning mrpt_bayes,
  mrpt_containers, mrpt_core, mrpt_graphs, mrpt_graphslam, mrpt_hwdrivers,
  mrpt_img, mrpt_maps, mrpt_math, mrpt_nav, mrpt_obs, mrpt_poses,
  mrpt_serialization, mrpt_slam, mrpt_system and mrpt_viz.
  Real bugs fixed along the way (not just silenced casts):
  * CPose3DQuat::operator[](i) const returned a temporary `double` by
  value while its `const_iterator` bound it to a `const double&`,
  making every dereference of a const iterator dangling
  (-Wreturn-local-addr).
  * CDirectedTree::Visitor and every one of its subclasses had virtual
  functions but a non-virtual destructor.
  * CSerializable's schema-archive overloads of serializeTo/serializeFrom
  were silently hidden in any class using DEFINE_SERIALIZABLE, so they
  were unreachable without explicit qualification; added the missing
  `using` declarations.
  * mrpt::Clock::time_point had no discoverable (ADL-reachable)
  operator<<, which made gtest's EXPECT_EQ/ASSERT_EQ fail to compile
  wherever it compared two timestamps on gcc-11/older libstdc++ (the
  actual Humble build error, not just a warning) -- this was the one
  case where mrpt::system::operator<<(TTimeStamp) already existed but
  lived in a namespace ADL never finds from outside mrpt::system; it is
  now `mrpt::operator<<`, with `mrpt::system` bringing it back in via a
  `using` declaration for existing qualified callers.
  * Several TKFMethod-shaped unscoped enums (TKFMethod,
  TLaserSimulUncertaintyMethod, TMatrixTextFileFormat,
  TICPCovarianceMethod) had no fixed underlying type, so a test
  casting an intentionally-invalid value into them triggered
  "conversion is unspecified" rather than the well-defined reinterpret
  a fixed underlying type gives.
  Everything else is either a widening/narrowing cast made explicit, an
  unused parameter renamed to a comment, `else`-ambiguous one-liners
  given braces, or a documented false-positive (e.g. a wrapper around a
  cached-static-map accessor gcc's -Wdangling-reference still flags)
  resolved by dropping the unnecessary named reference instead of
  suppressing the warning.
  Verified with an incremental colcon build of all 16 touched packages
  under mrpt_common's warning set (-Wall -Wextra -Wshadow -Wtype-limits
  -Wcast-align -Wparentheses -Wunused -Wpedantic -Wconversion
  -Wsign-conversion -Wdouble-promotion -Woverloaded-virtual
  -Wold-style-cast -Wnon-virtual-dtor): zero warnings left other than a
  gcc-13/-O3 -Wmaybe-uninitialized false positive on small fixed-size
  CMatrixDynamic construction that isn't present in either buildfarm log
  and isn't touched by this change. All existing unit tests
  (2000+ cases across the touched packages) still pass.
  Claude-Session: https://claude.ai/code/session_01PvVpfLXnmP5wfAzpEjd7vJ
  * fix: replace the Clock::time_point operator<< with a gtest PrintTo hook
  The operator<<(Clock::time_point) added in the previous commit, found via
  ADL because mrpt::Clock is in namespace mrpt, broke unrelated `os << x`
  calls in any translation unit with several `using namespace mrpt::...;`
  directives active at once (e.g. MonteCarloLocalization_App.cpp, which
  has ~15 of them): overload resolution for `os << particles_count`
  (a std::vector<int>) stopped finding mrpt::math's own
  operator<<(ostream&, const std::vector<T>&) entirely -- this broke all
  four CI compilers (gcc, clang, AppleClang, MSVC), all failing on the
  exact same line.
  It was also never the right fix for a second reason: mrpt::system
  already had an operator<<(TTimeStamp) doing the same thing (TTimeStamp
  being just an alias of Clock::time_point), so having both in scope at
  once was flat-out ambiguous the moment `using namespace mrpt::system;`
  and ADL both applied -- which is exactly the CObservation.cpp failure
  this was meant to fix in the first place.
  gtest's own recommended mechanism for exactly this situation --
  teaching it to print a type it doesn't otherwise know how to -- is a
  `PrintTo(value, ostream*)` overload, looked up via ADL *before* gtest
  falls back to operator<<. It doesn't add anything to the type's normal
  operator<< overload set, so it cannot participate in, or break, any
  unrelated overload resolution. mrpt::system::operator<<(TTimeStamp) is
  reverted to its original, unmodified form.
  Verified with a clean incremental build of all 21 packages (the 16 from
  the previous commit plus mrpt_libapps_gui/_cli, mrpt_gui, mrpt_apps_gui/_cli,
  which pulled in MonteCarloLocalization_App.cpp): zero errors, zero
  warnings. The four originally-affected unit test binaries
  (mrpt_core, mrpt_system, mrpt_containers, mrpt_obs) still pass in full.
  Claude-Session: https://claude.ai/code/session_01PvVpfLXnmP5wfAzpEjd7vJ
  * TKFMethod enum class defined as uint8_t
* Merge pull request `#1408 <https://github.com/MRPT/mrpt/issues/1408>`_ from MRPT/feat/coverage-viz-legacy-serialization
  viz: cover the legacy serialization branches, fixing a .3Dscene load failure
* viz: actually exercise the legacy clip-distance reset
  Review follow-up: the Viewport test dirtied only the viewport clip pair, and
  asserted clip values solely for v >= 9, so the reset added for older streams
  had no coverage and a regression there would have passed. Both clip pairs are
  dirtied now, and both are asserted for every version.
  Verified by temporarily removing the reset: the test fails with the dirtied
  7/8 leaking through instead of the 0.01/1000 defaults.
  Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_014Mjmvqb883MdKyobSADXvJ
* viz: reset the remaining legacy-omitted fields on old streams
  Review follow-up: the previous commit reset some of the fields that an older
  stream omits but not all of them, in the very same two functions.
  Viewport::serializeFrom() also left m_background_color (absent before v1) and
  m_OpenGL_enablePolygonNicest (absent before v2) untouched, and
  COctoMapVoxels::serializeFrom() skipped the whole triangle-params blob before
  v3, leaving m_enableLight and m_cullface stale. CompiledViewport and the
  triangle render proxies read all four, so a reused object carried its previous
  appearance into the loaded one.
  The tests now dirty every such field before reading the legacy frame, so a
  missing reset fails the assertion instead of passing by accident.
  Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_014Mjmvqb883MdKyobSADXvJ
* viz: extend the legacy serialization coverage to CArrow, CAxis and COctoMapVoxels
  COctoMapVoxels left m_color_map untouched when reading a pre-v4 stream, so an
  old file loaded into a reused object kept whatever colormap it had instead of
  the cmHOT default — the same stale-field pattern already fixed in Viewport.
  The other two classes' legacy branches turned out to be correct; the tests
  pin them down, including the fields each version dropped (CArrow's v1
  roll/pitch/yaw and CAxis's single pre-v1 tick-mark flag).
  Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_014Mjmvqb883MdKyobSADXvJ
* viz: cover the legacy serialization branches, fixing a Viewport load failure
  Adds tests/legacy_serialization.h (the helper from mrpt_math/mrpt_obs, plus a
  writer for the CVisualObject render header, which is versioned independently
  of each class) and drives the backwards-compatibility branches of Viewport
  (v0..v10), CCamera (v0..v5), CPointCloud (v0..v7), CSetOfLines (v0..v4),
  CPointCloudColoured, CSphere and CBox, none of which any test had reached.
  Bugs found:
  * Viewport::serializeFrom() read the "has image-view plane" flag added in v5
  unconditionally, without the version guard every other field there has. Any
  stream holding a viewport older than v5 — i.e. any .3Dscene file written by
  a correspondingly old MRPT — desynchronized at that point and failed to load
  with an EOF error.
  * The same function left the clip distances (pre-v9) and the viewport
  visibility flag (pre-v10) untouched when they were absent from the stream,
  so loading an old viewport into a reused object silently kept the previous
  values. Both fall back to their documented defaults now.
  * CVisualObject::castShadows(bool doCast = true) defaulted its argument, which
  made the no-argument castShadows() resolve to the *setter* on any non-const
  object: the getter was unreachable there, and a caller reading the property
  would silently enable shadow casting instead. CVisualObject_unittest.cpp had
  already worked around it via a const alias. The default is dropped; the
  setter still takes an explicit argument.
  Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_014Mjmvqb883MdKyobSADXvJ
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
* viz: cover the JSON-scheme archives and COctoMapVoxels
  The scheme (JSON) serialization of CArrow, CCylinder and CPointCloud is a
  separate code path from the binary CArchive one and had never been
  exercised, including its unknown-version rejection.
  COctoMapVoxels' population API is protected, for use by the map classes
  that build the visualization, so the test drives it through a small derived
  helper: grid cubes, several voxel sets, per-set visibility, the solid-cube
  and points display modes, and the bounding box.
  Also drops CPointCloud's m_minmax_valid / m_col_slop members, which no
  longer had a reader after updateBuffers() was restored, and fixes an
  inverted "is this label outdated" test in
  CTextMessageCapable::regenerateGLobjects() that made it skip exactly the
  labels needing regeneration.
  Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_014Mjmvqb883MdKyobSADXvJ
* viz, opengl: apply clang-format
  Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_014Mjmvqb883MdKyobSADXvJ
* viz: cover CCamera and the viewport ray casting, fixing a lost camera pose
  New tests for CCamera's projection modes, pinhole model and serialization,
  and for Viewport::get3DRayForPixelCoord() in its projective, 6-DOF and
  orthogonal variants.
  CCamera::serializeTo() never wrote the base CVisualObject state nor the
  6-DOF flag, so a camera placed with set6DOFMode(true) + setPose() lost both
  its mode and its pose whenever the scene was saved and reloaded.
  Serialization version bumped to 5, still reading v0..v4 streams.
  Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_014Mjmvqb883MdKyobSADXvJ
* viz: cover the mesh, container and point-cloud APIs, fixing six bugs
  New MiscVisualObjects2_unittest.cpp exercises the classes whose only
  coverage was a serialization smoke test: CCylinder, CPointCloud,
  CSetOfObjects, CSetOfTriangles, CSetOfTexturedTriangles, CMesh, CMeshFast,
  CMesh3D, CVectorField3D, CColorBar, CTextMessageCapable, CText and CText3D.
  Bugs it found:
  * CPointCloud::setAllPointsFast() and
  CSetOfTriangles::insertTriangles(const Ptr&) each took their object's
  write lock and then called a method that locks the same non-recursive
  mutex again, so both deadlocked on every call.
  * CPointCloud never overrode updateBuffers(), so its color buffer stayed
  empty: enableColorFromX/Y/Z() and setGradientColors() had no effect and
  clouds always rendered in the flat object color.
  * CSetOfObjects::internalBoundingBoxLocal() unioned the children's *local*
  boxes, ignoring each child's own pose inside the set, so a composite
  object reported a box that did not contain its own geometry. Viewport
  already composes child poses the same way this now does.
  * CMesh::adjustGridToImageAR() and CMeshFast::adjustGridToImageAR() used
  width/height where height/width was meant, stretching the grid along the
  wrong axis and distorting the texture instead of fitting it.
  * CVectorField3D did not serialize its module-based color mapping settings
  (still/max-speed colors, max speed, and the two display flags), which
  were silently reset to defaults on load. Serialization version bumped to
  2, still reading v1 streams.
  Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_014Mjmvqb883MdKyobSADXvJ
* viz: fix PLY color handling, plus a point-cloud corruption on save
  CPointCloudColoured::PLY_export_get_vertex() had its assignments the wrong
  way round: instead of reading a point out of the cloud, it wrote the
  (uninitialized) output arguments into it. Saving a coloured point cloud to
  PLY therefore zeroed every point of the cloud in memory and wrote a file
  full of zeros. The existing round-trip test passed because it compared the
  already-clobbered source against the equally empty reloaded copy.
  While covering the rest of the PLY layer:
  * Per-channel "red"/"green"/"blue" vertex properties are now both read and
  written. The importer already had a branch for them but never requested
  them from the file, so it was dead code and every colored PLY file from
  another tool lost its color on load. Integer channels are scaled from
  [0,255], float ones taken as [0,1], and RGB now takes precedence over the
  grayscale "intensity" channel MRPT keeps writing for compatibility.
  * PlyProperty's constructor silently dropped its is_list argument, so the
  face element was written as "property int vertex_indices" instead of
  "property list uchar int vertex_indices".
  * Header comments and obj_info came back with their trailing newline
  attached, since mrpt::system::trim() only strips spaces and tabs.
  * The exporter left pt_has_color uninitialized before asking the object for
  a vertex, and CGenericPointsMap only ever set it to true, so a colorless
  generic point map wrote a nondeterministic header.
  New tests read PLY files written the way other tools write them (uchar and
  float color channels, double coordinates, a face element, a malformed
  header), and the maps-side round-trip test now checks full color instead of
  the previous grayscale-only behavior.
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
* viz: cover CAssimpModel/CAnimatedAssimpModel, fixing four bugs found
  New unit tests for the two Assimp-based classes, which had no coverage at
  all. Models are synthesized on the fly (Wavefront OBJ/MTL plus a PNG
  texture) except for the skinned/animated one, added as a small glTF 2.0
  file in the test data directory.
  Bugs found and fixed while writing them:
  * CAssimpModel::loadScene() always OR'ed aiProcess_GenSmoothNormals into
  the Assimp flags, which is mutually exclusive with the
  aiProcess_GenNormals implied by the "RealTimeFast" preset. Loading with
  LoadFlags::RealTimeFast therefore always failed.
  * CAssimpModel::serializeFrom() forwarded its own version number to
  CSetOfObjects::serializeFrom(), which only knows version 0, so any
  stream holding a (current, v1) CAssimpModel could not be read back.
  * CSetOfTriangles::updatePolygons() assigned the scratch polygon to the
  output inside the per-vertex loop; building a TPolygonWithPlane out of a
  partially filled triangle throws "points are aligned", which made
  traceRay() fail on common meshes.
  * CSetOfTriangles::getPolygons() wrote into the output vector without
  resizing it first.
  * CAnimatedAssimpModel was missing from registerAllClasses(), so it could
  never be deserialized.
  The serialization round-trip test now also covers every registered
  mrpt::viz class instead of a subset, which is what would have caught the
  two serialization defects above.
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
* Contributors: Jose Luis Blanco-Claraco

3.1.2 (2026-07-07)
------------------

3.1.1 (2026-07-04)
------------------

3.1.0 (2026-07-03)
------------------
* chore: clean warnings
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

