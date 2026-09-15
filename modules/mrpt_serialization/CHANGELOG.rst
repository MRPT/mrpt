^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_serialization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge pull request `#1413 <https://github.com/MRPT/mrpt/issues/1413>`_ from MRPT/feat/api-cleanups-3.1
  feat: API cleanups before the next minor bump (archives, const-correctness, points-map accessor)
* fix: address review feedback on the API cleanups
  * CArchive rvalue forwarders: probe and return the deduced archive type
  instead of CArchive, so that insertion/extraction operators declared
  only for a derived archive type are found, and chaining stays on that
  type. Covered by a new unit test with a derived-archive-only operator.
  * robot-map-gui: keep CDocument::config() const, handing out a list of
  CMetricMap::ConstPtr (the widget only reads map options), instead of
  exposing the child map list of a const multi-map.
  * Porting guide: drop the duplicate "removed" anchor.
* feat(mrpt_serialization): allow streaming into temporary archives
  Free operator<< / operator>> take the archive by non-const reference, so
  the temporary returned by archiveFrom() could not bind to them and users
  had to name an intermediate variable.
  Add rvalue-archive forwarders, mirroring what the standard library does
  for std::basic_ostream / std::basic_istream ([ostream.rvalue]): they
  forward to the regular lvalue overloads and return an lvalue reference so
  chaining keeps working, and they are SFINAE-constrained to archives whose
  equivalent lvalue expression is well-formed.
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
* Merge branch 'develop' into fix/stereo-rectify-map-axis-swap
* Contributors: Jose Luis Blanco-Claraco

3.1.4 (2026-09-04)
------------------
* test(mrpt_serialization): add a unit test suite (module had none) covering CArchive, CMessage, and STL/std::optional serialization (`#1390 <https://github.com/MRPT/mrpt/issues/1390>`_).
* fix(mrpt_serialization): CMessage::sendMessage()/receiveMessage() used mismatched byte order for the payload length (no message >= 256 bytes could be received) and misread an empty payload as a framing error; sendMessage() now also rejects payloads too large for the 16-bit length field instead of overflowing the frame buffer.
* fix(mrpt_serialization): operator<<(std::monostate) writes no version byte, but the reader only skipped it for "nullptr", so an empty std::variant failed to deserialize.
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

