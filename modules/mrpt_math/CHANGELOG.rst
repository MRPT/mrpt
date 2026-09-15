^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_math
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
* Merge pull request `#1411 <https://github.com/MRPT/mrpt/issues/1411>`_ from MRPT/feat/coverage-img-math-blindspots
  Coverage blind spots in mrpt_img and mrpt_math: reconnect SIMD, remove dead code, fix 6 bugs
* fix(mrpt_math): remove dead headers, fix 4 geometry/IO bugs, raise coverage
  Dead code removed (~1720 lines). The public headers CBinaryRelation.h,
  matrix_adaptors.h, MatrixBlockSparseCols.h, CMonteCarlo.h and
  eigen_extensions.h are included by nothing in the repository, and all but
  eigen_extensions.h fail to even compile standalone: they reference
  CMatrixTemplateObjects (removed in the 3.x matrix rewrite) or Eigen::Matrix
  without including Eigen. No working downstream code can be using them.
  Also removed a second, unreferenced
  ::intersect(TPolygonWithPlane, TPolygonWithPlane, TObject3D) in geometry.cpp,
  duplicating what intersectAux() already does for the public
  intersect(TPolygon3D, TPolygon3D).
  Bugs fixed:
  * assemblePolygons()'s three TObject3D overloads collected the polygons
  already present in the input and then called the segment-based overload,
  which overwrites its output vector: every pre-existing polygon was
  silently dropped.
  * assemblePolygons(segments, ...) looped `for (size_t i = 0; i < N - 1; i++)`,
  which underflows to ~2^64 iterations for an empty input -- a hang, not a
  wrong answer.
  * TSegment3D::distance(TPoint3D) returned
  min(d(p,p1), d(p,p2), d(p, infinite line)), so a point beyond an endpoint
  measured to the unbounded supporting line: a point 1 m past the end of a
  segment reported distance 0. It now clamps the projection parameter to
  [0,1], like its correct 2D twin TSegment2D::signedDistance().
  distance(TSegment3D) additionally mishandled a zero-length operand (the
  "almost parallel" branch settles on the wrong endpoint) and now delegates
  to the point overload.
  * MatrixVectorBase::saveToTextFile() wrote userHeader with no trailing
  newline although its documentation says "final end-of-line is not needed",
  gluing the header onto the first data row. With the usual %-prefixed
  header that made loadFromTextFile() skip the first row of data silently.
  Doc correction: saveToTextFile()'s appendMRPTHeader text described a header
  string the code has not written for years.
  New tests cover the container API corner cases (generic iterators, linear
  indexing, data-preserving setSize() including its non-trivial element-type
  branch, text-file I/O error paths), the degenerate branches of
  getAngleBisector(), TLine3D, assemblePolygons(), splitInConvexComponents(),
  polygon/plane intersection, and the special functions in math.cpp and
  poly_roots.cpp.
  Line coverage 95.0% -> 96.8%.
  Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01Byu4YEBNCgqhysGdjCeVTQ
* Merge pull request `#1407 <https://github.com/MRPT/mrpt/issues/1407>`_ from MRPT/feat/coverage-viz-opengl
  Coverage pass on mrpt_viz: 70% -> 87%, fixing four classes that rendered nothing
* Keep TLine3D::distance() honest about a degenerate director
  The previous clamp used std::max(0, radicand), which turns the NaN produced
  by a zero-length director into a distance of 0.0 — a wrong answer reported
  as a valid one, and passing a NaN to std::max is UB anyway. Comparing the
  radicand against zero instead only rescues a genuinely negative one (the
  rounding case this was written for) and lets the degenerate case stay NaN.
  Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_014Mjmvqb883MdKyobSADXvJ
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
* Merge pull request `#1406 <https://github.com/MRPT/mrpt/issues/1406>`_ from MRPT/feat/coverage-math-maps-obs-slam
  Coverage pass on mrpt_math, mrpt_maps, mrpt_obs and mrpt_slam
* Fix the MSVC build: M_PI is not defined without an explicit include
  Windows CI failed with "error C2065: 'M_PI': undeclared identifier" in
  fresnel_unittest.cpp. MSVC does not define the M_PI family unless
  _USE_MATH_DEFINES is set before <cmath>; MRPT provides them from
  mrpt/core/bits_math.h (M_PI) and mrpt/core/common.h (M_PIf), which the
  other tests were picking up transitively. Both new tests that use them
  now include those headers explicitly rather than relying on that.
  Also compare getFileSize()'s uint64_t result against an unsigned literal.
* Assert every coordinate vector in the over-requested 3D kNN test
  The kdTreeNClosestPoint3DWithIdx case checked only xs, so a regression
  leaving ys/zs sized at knn would have passed.
* Address review: fix a heap overflow in CMatrixDynamic::realloc and trim KNN results
  macOS CI aborted in the new CMatrixDynamic.resizeOverloads test, which
  turned out to expose a real out-of-bounds write: when a matrix shrinks in
  rows while growing in columns with zeroNewElements=true (e.g. 5x1 -> 2x4),
  realloc()'s "zero the new columns" pass iterated over the *old* row count,
  writing past the end of the new, smaller buffer. macOS's hardened
  allocator aborts on it; glibc silently tolerated it. The loop is now bound
  by the number of surviving rows. The sibling "zero the new rows" memset
  also covered only one cell per row instead of a whole row.
  Also from the review:
  * KDTreeCapable's ordinary (non radius-limited) k-NN searches left their
  output vectors sized at knn even when the cloud holds fewer points, so
  callers received uninitialized trailing indexes and distances. Same fix
  as the radius-limited path, now covered by a test that asks for more
  neighbors than exist.
  * The angle-bisector regression test used coefficients for which the two
  bugs it is meant to pin cancel out, so it passed against the unfixed
  code; it now uses a line whose normalization factor differs from |C|.
  * The point-map fusion test asserted an upper bound that unfused
  insertion also satisfies, and its 3D fixture put a point at the origin,
  which is dropped as an invalid reading; both are now strict.
  * The data-association tests assert the actual observation-to-prediction
  pairing rather than only the count, which a permutation would pass.
  * The Fresnel reference integration used 2e6 Simpson intervals; 2e4 keeps
  the error far below the assertion tolerance and takes the whole
  mrpt_math suite from ~700ms to ~110ms.
* Coverage pass on mrpt_math, mrpt_maps, mrpt_obs and mrpt_slam
  Raises line coverage above the 90% goal in all four modules:
  mrpt_math 88.8% -> 95.0%, mrpt_maps 86.5% -> 90.2%,
  mrpt_obs 88.5% -> 91.6%, mrpt_slam 90.2% -> 93.5%.
  A new test helper (tests/legacy_serialization.h, in mrpt_math and
  mrpt_obs) writes an MRPT object frame with an arbitrary streaming
  version, so the backwards-compatibility branches of serializeFrom()
  can be exercised without binary fixture files.
  Bugs found and fixed along the way:
  * TLine3D::TLine3D(const TLine2D&) divided by the coefficient it had
  just tested for zero when converting a horizontal 2D line, yielding
  an infinite/NaN base point.
  * getAngleBisector(TLine2D, TLine2D)'s parallel-lines branch used the
  wrong coefficient when normalizing the second line, and never halved
  the resulting offset; the two errors cancelled only for the case the
  pre-existing test happened to use.
  * The templated intersect() over vectors of geometric objects iterated
  the inner loop up to the first vector's size, reading out of bounds,
  and its std::vector overload took the output container by value.
  * KDTreeCapable's radius-limited k-NN searches left the coordinate
  output vectors sized at knn while trimming the index/distance ones,
  handing back stale entries.
  * CHistogram::createWithFixedWidth() was not static, so the documented
  usage (and its only caller, CMonteCarlo) did not compile.
  * KLD_Gaussians() called a non-existent inverse_LLt() overload and the
  row-vector form of the quadratic-form helper; it was uninstantiable.
  * CGasConcentrationGridMap2D::simulateAdvection() read past the end of
  an empty matrix for any map type other than mrKalmanApproximate; it
  now returns false with an error log.
* Merge branch 'develop' into fix/stereo-rectify-map-axis-swap
* Contributors: Jose Luis Blanco-Claraco

3.1.4 (2026-09-04)
------------------

3.1.3 (2026-08-12)
------------------
* mrpt_math: depend on nanoflann_vendor instead of nanoflann
  The nanoflann rosdep key resolves to the distro's own libnanoflann-dev,
  so it can never select a newer vendored version. Switch to the new
  nanoflann_vendor ROS package name.
* Contributors: Jose Luis Blanco-Claraco

3.1.2 (2026-07-07)
------------------
* Fix ``nanoflann`` declared as a build-only dependency in ``package.xml``,
  which prevented it from being found by downstream packages' CMake
  configuration on ROS build farms.
* Increase unit test coverage.
* Contributors: Jose Luis Blanco-Claraco

3.1.1 (2026-07-04)
------------------
* math: add nanoflann as rosdep dependency (more up-to-date than system libnanoflann-dev)
* Contributors: Jose Luis Blanco-Claraco

3.1.0 (2026-07-03)
------------------
* Merge pull request `#1376 <https://github.com/MRPT/mrpt/issues/1376>`_ from MRPT/port/2x-fixes-jul2026
  Port 2.x fixes: COutputLogger thread-safety, octomap build flag
* Merge pull request `#1374 <https://github.com/MRPT/mrpt/issues/1374>`_ from MRPT/feat/kdtree-save-load-index
  feat(math): KDTreeCapable save/load of the KD-tree index (2D and 3D)
* Contributors: Jose Luis Blanco-Claraco

3.0.4 (2026-06-17)
------------------

3.0.3 (2026-06-15)
------------------
* Increase unit test coverage for mrpt_math, mrpt_io, mrpt_system (`#1365 <https://github.com/MRPT/mrpt/issues/1365>`_)
* Merge pull request `#1364 <https://github.com/MRPT/mrpt/issues/1364>`_ from MRPT/fix/rdev-test-failures-3.0.3
  Fix/rdev test failures 3.0.3
* fix: alignment crash
* Contributors: Jose Luis Blanco-Claraco

3.0.2 (2026-06-11)
------------------

3.0.1 (2026-06-11)
------------------
* Merge pull request `#1363 <https://github.com/MRPT/mrpt/issues/1363>`_ from MRPT/fix/dont-export-eigen3-dep
  refactor: limit visibility of eigen3 as build dep
* fix: KF math errors
* refactor: limit visibility of eigen3 as build dep
* Contributors: Jose Luis Blanco-Claraco

3.0.0 (2026-06-06)
------------------

2.20.0 (2026-06-06)
-------------------
* Last release of the 2.x series. Starting from 3.0.0, changes are tracked
  in each module's own CHANGELOG.rst file.
* Contributors: Jose Luis Blanco-Claraco

