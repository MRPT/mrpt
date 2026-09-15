^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_hwdrivers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Merge pull request `#1409 <https://github.com/MRPT/mrpt/issues/1409>`_ from MRPT/feat/coverage-hwdrivers-mocked-streams
  hwdrivers: mocked-stream coverage pass (13.9% -> 29.0%), reviving dead pcap support and fixing 7 defects
* hwdrivers: address review feedback on the new tests
  - CSickLaserSerial `nonMeasurementFramesAreIgnored` was passing for the wrong
  reason: it patched the command byte after makeScanFrame() had already
  computed the CRC over it, so the driver rejected the frame at the CRC check
  and never reached the command-byte dispatch the test is named after. The
  command byte is now a parameter, so the CRC covers it.
  - C2DRangeFinderAbstract `repeatedMissingScansEventuallyReportAFailure` used a
  fixed 80 iterations to drive the scan-period estimator down, which a loaded
  machine could leave above the bound. Loop until it converges instead, and
  derive the quiet interval from the measured period.
  - CHokuyoURG_unittest.cpp uses ::snprintf and M_PI, so include <cstdio> and
  <cmath> rather than relying on transitive includes.
  Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_014Mjmvqb883MdKyobSADXvJ
* hwdrivers: retry GPS setup commands after a failed attempt
  Review feedback: tryToOpenTheCOM() latched m_setup_cmds_sent before knowing
  whether OnConnectionEstablished() succeeded, so a write error while sending
  the setup commands to an externally bound stream left the receiver
  unconfigured with no further attempt. Latch only on success, which is what
  the serial-port path above already does by reopening the port.
  Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_014Mjmvqb883MdKyobSADXvJ
* hwdrivers: smoke-test the sensor factory, fixing four destructor crashes
  Adds tests for the CGenericSensor factory: name lookup, unknown/empty class
  names, the common config parameters, and a round-trip over every driver that
  registerAllClasses_mrpt_hwdrivers() registers.
  That last one constructs and destroys each registered driver without calling
  initialize(), which is exactly what rawlog-grabber does between creating a
  sensor by class name and configuring it. Four drivers aborted the process on
  that path, because their destructor used a resource that only initialize()
  creates:
  - ~CIbeoLuxETH() joined a thread that was never started, and a std::system_error
  escaping a destructor calls std::terminate().
  - ~CRoboPeakLidar() called turnOff(), which throws when built without the
  RPLidar SDK.
  - ~CImpinjRFID() wrote to a null client socket.
  - ~CGyroKVHDSP3000() closed a null serial port.
  Each now checks before acting. Drivers whose optional SDK is missing still
  refuse construction with their existing descriptive exception; the test
  accepts that and requires the rest to build and tear down cleanly.
  Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_014Mjmvqb883MdKyobSADXvJ
* hwdrivers: cover the SICK LMS serial scan decoder
  CSickLaserSerial advertises bindIO(), but its frame reader, ACK waiter and
  command sender each did a dynamic_cast to CSerialPort and asserted on it, so
  any other bound stream aborted at the first read. Those four functions only
  ever call Read()/Write(), so they now go through the bound stream directly;
  the remaining CSerialPort uses (open, setConfig, purgeBuffers) are genuinely
  serial-specific and stay as they were.
  That makes the driver testable, and the new tests cover the measurement frame
  decoder over a mocked stream: centimeter and millimeter modes, out-of-range
  readings, CRC rejection, resynchronization after garbage, non-measurement
  frames, and the silent-device and unconfigured cases (0% -> 32% lines).
  Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_014Mjmvqb883MdKyobSADXvJ
* hwdrivers: cover the CGPSInterface driver layer over a mocked stream
  The existing tests only exercised the NMEA/NOVATEL parsers. These drive the
  driver itself through a bound MockStream: frame dispatch, the GGA cache,
  resynchronization after garbage, setup/shutdown commands, custom commands,
  parser selection, sensor-label decoration and loadConfig (19% -> 47% lines).
  Two defects surfaced:
  - OnConnectionShutdown(), called from the destructor, dereferenced a null
  m_data_stream whenever shutdown commands were configured but no stream was
  ever opened, e.g. when the object is destroyed right after loadConfig().
  It now bails out when there is nothing connected.
  - Setup commands were only sent from the serial-port open path, so with an
  externally bound stream they were silently dropped while their shutdown
  counterparts were still sent. They are now sent once for external streams
  too.
  Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_014Mjmvqb883MdKyobSADXvJ
* hwdrivers: cover the 2D laser scanner drivers with a mocked I/O stream
  Adds a MockStream test helper (a CStream that records what a driver writes
  and replays scripted device answers) so request/response sensor protocols
  can be exercised with no hardware attached, and uses it for:
  - C2DRangeFinderAbstract: observation hand-off, error state, exclusion
  areas/angles, and the missed-scan failure counter (0% -> 99% lines).
  - CHokuyoURG: the full SCIP2.0 turn-on handshake, scan decoding, and the
  truncated-scan/out-of-range/silent-device paths (0% -> 62% lines).
  Two defects surfaced while writing these:
  - C2DRangeFinderAbstract::getObservation() could never return anything:
  m_lastObservation, m_lastObservationIsNew and m_hardwareError were read
  but never written. doProcess() now records them, and getObservation()
  clears the "new" flag as its documentation promises.
  - CHokuyoURG::parseResponse() only validated the device status code on
  replies that carry data, so an error answer to a status-only command
  (BM, QT, CR, ...) was silently taken as success.
  Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_014Mjmvqb883MdKyobSADXvJ
* hwdrivers: revive the Velodyne pcap support and its tests
  The libpcap detection was incomplete in the modular build, so the offline
  (.pcap replay) half of CVelodyneScanner was compiled out everywhere:
  - `find_package(PCAP)` had no FindPCAP.cmake to work with, so
  CMAKE_MRPT_HAS_LIBPCAP was always 0.
  - MRPT_HAS_LIBPCAP was never emitted into hwdrivers' config.h.
  - CVelodyneScanner.cpp did not include that config.h anyway, so the
  guard read an undefined macro.
  - The unit test additionally guarded on MRPT_HAS_TINYXML2 without
  including mrpt/obs/config.h, where that macro lives.
  With the four fixed together, both CVelodyneScanner tests run (and pass)
  against the sample .pcap datasets instead of silently compiling to nothing.
  Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_014Mjmvqb883MdKyobSADXvJ
* Merge branch 'develop' into fix/stereo-rectify-map-axis-swap
* Contributors: Jose Luis Blanco-Claraco

3.1.4 (2026-09-04)
------------------

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

