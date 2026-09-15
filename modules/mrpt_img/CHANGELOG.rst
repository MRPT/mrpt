^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_img
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* feat(mrpt_img): reconnect the SIMD kernels and close coverage blind spots
  The SSE2/SSSE3 kernels in CImage.SSE2.cpp / CImage.SSSE3.cpp had been
  orphaned by the stb-based CImage rewrite: nothing but their own unit test
  called them, and scaleHalf()/grayscale() documented their bool return as
  "always false, reserved for a future SIMD fast path".
  scaleHalf() now dispatches to image_SSSE3_scale_half_3c8u (3-channel
  IMG_INTERP_NN), image_SSE2_scale_half_1c8u (1-channel NN) and
  image_SSE2_scale_half_smooth_1c8u (1-channel IMG_INTERP_LINEAR), and
  grayscale() to image_SSSE3_rgb_to_gray_8u. The bool return now truthfully
  reports whether a fast path ran. The gray kernel asserts both row strides
  are multiples of 16 bytes, which for CImage's packed rows means
  width % 16 == 0; none of the kernels can run in place. This also restores
  MRPT 2.x's IMG_INTERP_NN semantics for scaleHalf (point sampling, where the
  stb STBIR_FILTER_BOX path box-averages).
  Both the vectorized and the portable branch are asserted against each other
  using mrpt::cpu::overrideDetectedFeature(), so neither depends on the host
  CPU for coverage.
  Bugs fixed:
  * CImage::grayscale(ret) documents in-place use (ret = *this), but
  ret.resize() freed the source buffer before the conversion loop read it:
  garbage output plus a 2-byte heap over-read on the last pixel.
  * TCamera::serializeFrom() did not reset cameraName for pre-v5 streams,
  unlike nrows/ncols/distortion in the same function, so reading a legacy
  camera into a reused object kept the previous name.
  Doc corrections: the scaleHalf()/grayscale() return-value contracts, and
  the SIMD kernels' doxygen, which still pointed at CImage::scaleHalfSmooth()
  and CImage::grayscaleInPlace() -- neither exists in 3.x.
  New tests cover the MRPT 2.x CImage serialization formats (v0..v9 and v100)
  via a legacy_serialization.h helper, TCamera/TStereoCamera legacy formats,
  the generic CCanvas::drawImage() implementation, the DistortionModel::none
  and kannala_brandt branches of camera_geometry, XPM hexadecimal and named
  colors, and the RGBA load/scale paths.
  Line coverage 93.3% -> 97.9%, branches 71.9% -> 77.2%.
  Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01Byu4YEBNCgqhysGdjCeVTQ
* Merge pull request `#1404 <https://github.com/MRPT/mrpt/issues/1404>`_ from MRPT/fix/stereo-rectify-map-axis-swap
  fix(img): CStereoRectifyMap scrambled the forward axis for horizontal-baseline stereo pairs
* Merge branch 'develop' into fix/stereo-rectify-map-axis-swap
* img: fix the rectified X axis direction in CStereoRectifyMap
  setFromCamParams() derived the rectified frame's e1 from T = -R*T_fwd, which
  is the position of the LEFT camera in the RIGHT camera's frame, instead of
  T_fwd, the right camera in the left's. For an ordinary side-by-side rig that
  flips e1 and, with it, e2: the rectification comes out rotated 180 deg
  in-plane and the right camera lands at NEGATIVE x in the rectified frame, so
  every disparity has the wrong sign. Both rectified images flip together, so
  the pair stays epipolar-aligned - which is why the existing tests, checking
  output size or a marker at the principal point (the one point invariant under
  that flip), could not see it.
  Also report the rectified geometry in getRectifiedImageParams(): after
  rectification the pair is parallel with a pure baseline along +x, so
  rightCameraPose should say that rather than echo the input pose. Downstream
  code reads it to get the baseline to use with the rectified images.
  New test Rectify_idealPairIsIdentity: an already-rectified, distortion-free
  pair must rectify to itself. It checks the rectification quaternion, the
  reported rectified baseline, and an OFF-CENTER marker, which is what
  distinguishes the identity from the 180 deg flip.
  Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01DErBjLRU438kY9bKWYAaWo
* Address review: rotate the forward reference by R_half, tighten the
  degeneracy check
  e1 (and T_half) live in the half-rotated frame, but e3's forward
  reference was the bare, unrotated camera z axis - correct only when
  R_half happens to be near-identity. For a genuinely non-identity
  relative rotation between the two cameras, this derives e3 from the
  wrong frame, so the rectified center ray no longer maps back to the
  left camera's own forward direction. Fixed by rotating the reference
  (R_half * z, falling back to R_half * y) before projecting it
  orthogonal to e1, matching e1's own frame.
  Also tightened the degeneracy check itself: the old |e1.dot(z)| > 0.9
  threshold (~26 deg) misclassifies some non-degenerate baselines as
  needing the fallback - e.g. a baseline at (sqrt(1-0.91^2), 0, 0.91)
  still projects to a well-conditioned e3 (norm ~0.41) but was previously
  routed through the y-axis fallback regardless. Now checks the actual
  projected norm instead of the angle to z.
  New tests: Rectify_preservesForwardAxis_withRelativeRotation (the first
  issue, a real non-identity R_half) and
  Rectify_obliqueBaselineNearOpticalAxis (the second, a baseline close to
  but not exactly along the optical axis - checks the rectification
  rotation itself is a valid, finite unit quaternion and does not throw,
  since a baseline this close to the optical axis is a "forward motion"
  configuration where even a mathematically-correct rectification
  legitimately reprojects the principal point outside the original field
  of view - not the invariant the other two tests check). All 15
  CStereoRectifyMap tests and the 9 CUndistortMap tests still pass.
  Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01DErBjLRU438kY9bKWYAaWo
* clang-format
  Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01DErBjLRU438kY9bKWYAaWo
* fix(img): CStereoRectifyMap scrambled the forward axis for any
  horizontal-baseline stereo pair
  setFromCamParams() derived the new e2 (rectified "down") axis by
  crossing the baseline direction with a fixed reference "up" vector,
  then set e3 = e1 x e2. For an ordinary side-by-side stereo rig the
  baseline is itself close to the camera's own x axis, which this
  heuristic treats no differently from any other baseline direction: the
  original camera's z (forward, "looking direction") ends up assigned to
  e2, not e3. Since build_rectify_map() unprojects each rectified pixel
  by dividing by the z component of the rotated ray, every pixel of every
  image from a normal stereo pair rectified this way divides by ~0 and
  maps outside the source image, producing a blank (or near-blank)
  rectified output.
  Verified independently in Python (replicating the exact algorithm) and
  in C++: a 3D point directly in front of the original left camera, at
  z=3, transforms to (0, 3, 0) under the old e1/e2/e3 - its entire depth
  lands on the y axis. New test (Rectify_preservesForwardAxis) confirms
  this at the class's own level: a marker at the left camera's principal
  point, rectified through a plain 0.10 m x-baseline pair with zero
  relative rotation, previously vanished entirely (sumW hits the
  ASSERT_GT(sumW, 0.0) guard, i.e. the whole output is black); after the
  fix it reappears within 5 px of the rectified principal point. The
  existing Rectify_basic/_color tests did not catch this because they
  render a smooth gradient and only assert output image *dimensions* -
  which a rectification broken in exactly this way still satisfies.
  Fix: derive e3 (not e2) from the reference vector instead, using a
  Gram-Schmidt projection of the camera's own forward axis (0,0,1)
  orthogonalized against the baseline, falling back to (0,1,0) only in
  the genuinely degenerate case where the baseline itself is nearly
  colinear with the optical axis (a forward-facing, depth-separated
  pair). e2 = e3 x e1 keeps (e1, e2, e3) right-handed, matching the
  camera's own x-right/y-down/z-forward convention (x cross y = z).
  No test in this suite previously exercised CStereoRectifyMap's actual
  output geometry, only its plumbing (sizes, exceptions raised/not
  raised) - found while wiring a real fisheye stereo rig (GrandTour's
  Alphasense) through mola_vision, whose stereo mode requires a
  pre-rectified pair and has no rectification step of its own before this.
  Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01DErBjLRU438kY9bKWYAaWo
* Contributors: Jose Luis Blanco-Claraco

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

