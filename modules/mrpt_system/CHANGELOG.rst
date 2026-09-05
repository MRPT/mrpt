^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_system
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.1.4 (2026-09-04)
------------------
* Merge pull request `#1393 <https://github.com/MRPT/mrpt/issues/1393>`_ from MRPT/test/coverage-system
  test(mrpt_system): cover the untested helpers and the file system watcher
* Merge remote-tracking branch 'origin/develop' into test/coverage-comms
* Merge remote-tracking branch 'origin/develop' into test/coverage-system
* fix(mrpt_system): don't use CancelIoEx in the watcher destructor
  It is not declared with the SDK settings this build uses. Closing the
  notification handle already makes the watch thread's pending
  ReadDirectoryChangesW() return, which is what the join needs.
* fix(mrpt_system): join CFileSystemWatcher's watch thread on destruction
  On Windows the constructor starts a std::thread and the destructor only
  closed the notification handle, so destroying a successfully-constructed
  watcher hit std::thread's destructor with a joinable thread and called
  std::terminate(). The new unit tests are the first code to ever destroy one,
  and they took the whole test binary down with them.
  The pending ReadDirectoryChangesW() is now cancelled before the handle is
  closed, the thread is joined, and whatever it had queued but nobody read is
  freed instead of leaked.
  Also fixes the POSIX branch, which called inotify_rm_watch() with a
  descriptor it had already closed and reset to -1.
* Merge pull request `#1392 <https://github.com/MRPT/mrpt/issues/1392>`_ from MRPT/test/coverage-io
  test(mrpt_io): cover the stream, compression and path helpers
* test(mrpt_system): skip the missing-directory watcher test where unsupported
  Without inotify (macOS, and any other platform with no notification
  backend) CFileSystemWatcher's constructor is a no-op, so it never notices
  that the watched directory does not exist. Skip that expectation there, and
  pin the empty-path assertion, which does hold everywhere.
* test(mrpt_system): cover the untested helpers and the file system watcher
  Four files in this module had never had an assertion run against them
  (`CFileSystemWatcher.cpp`, `CObserver.cpp`, `progress.cpp`,
  `hyperlink.cpp`), and several others were only covered incidentally by other
  modules' tests.
  New test files:
  * `CFileSystemWatcher_unittest.cpp` watches a temporary directory and checks
  that file creation, deletion and sub-directory creation are all reported,
  skipping itself where the platform provides no notification support
  (0% -> 91%).
  * `CObserver_unittest.cpp` covers the observer/observable pair: publishing to
  several subscribers, unsubscribing, and the two destruction orders --
  observer first and observable first, the latter of which publishes
  mrptEventOnDestroy before dropping everyone.
  * `misc_system_unittest.cpp` covers progress(), hyperlink(), the CRC
  helpers, every thread and process priority level, and CRateTimer.
  * `os_unittest.cpp` covers the os:: C-library wrappers, getMRPTLicense(),
  executeCommand() (including its working-directory form), launchProcess()
  and the plug-in loader's failure paths.
  * `datetime_format_unittest.cpp` covers the timestamp formatters, the
  parts/timestamp round trip in both UTC and local time, and every unit
  branch of intervalFormat() (16.7% -> 97.0% for datetime.cpp).
  Bugs found and fixed:
  * `mrpt::system::consoleColorAndStyle()` had its stream selection inverted:
  `applyToStdErr = true` emitted the escape sequence on **stdout** and vice
  versa. COutputLogger passes that flag for its error-level messages, so the
  color codes landed on the opposite stream from the text they were meant to
  color. Its memo of the last colors applied was also shared between the two
  streams, which would suppress a needed escape sequence when alternating
  between them; it is now kept per stream.
  Module coverage: 66.0% -> 82.5% lines, 48.2% -> 62.3% branches.
* test(mrpt_io): cover the stream, compression and path helpers
  New test files for the parts of the module that had never been exercised
  directly: CStream's printf/printf_vector/getline, CMemoryStream's cursor and
  buffer management, the file input/output streams (including every error
  path), the compressed streams (a parameterized round-trip over None/Gzip/Zstd
  plus the CFileGZ* pair), the zip:: helpers, detect_compression, the lazy-load
  path helpers and CTextFileLinesParser.
  Bugs found and fixed:
  * `mrpt::io::CompressionType` was defined twice, identically, in
  `detect_compression.h` and in `compression_options.h`, both in namespace
  `mrpt::io`: including both headers in the same translation unit did not
  compile. `detect_compression.h` now includes the other one.
  * `zip::decompress()`'s std::vector overload passed an *uninitialized*
  length to zlib's `uncompress()`, which takes it as the capacity of the
  output buffer, so zlib was free to write past the end of the vector and
  the returned size was garbage. The three sibling overloads all initialize
  it correctly.
  * `mrpt::system::getFileSize()` is documented to return `size_t(-1)` when
  the file cannot be accessed, but used the throwing overload of
  `std::filesystem::file_size`. Every caller testing for `-1` --
  `CFileGZInputStream::open()` and, through it, `zip::decompress_gz_file()`,
  both documented to report failure by returning false -- threw instead.
  * `CMemoryStream::Seek()` used the `Origin` enumerator in place of `Offset`
  in its "from the end" case, so the requested offset was ignored entirely;
  it also clamped to the *allocated* size minus one, which underflowed to a
  huge value on an empty stream and made the end-of-data position
  unreachable. It now behaves like the file streams: offsets are relative to
  the chosen origin and clamped to [0, bytesWritten].
  * `CStream::getline()` left an unwritten byte in the output string when it
  hit the end of the stream, so the last line of a file without a trailing
  newline came back with a stray trailing null character.
  Left documented rather than changed: `CFileGZInputStream` and
  `CCompressedInputStream`'s single-argument constructors both document an
  exception on failure but ignore open()'s result, so they silently yield a
  closed stream, while their output counterparts do throw. Changing that would
  alter the contract of `zip::decompress_gz_file()`, which relies on the
  non-throwing behavior.
  Module coverage: 63.5% -> 82.5% lines, 43.5% -> 62.2% branches.
* Merge pull request `#1389 <https://github.com/MRPT/mrpt/issues/1389>`_ from MRPT/test/coverage-graphslam-system-io
  test(graphslam, system, io, slam): cover the largest untested pure-logic files, fix 8 bugs
* test(mrpt_system): drop a rate-dependent CControlledRateTimer assertion
  The "larger a0 keeps the estimate nearer the set-point" test only holds
  while the achieved rate is far from the set-point. On a machine that hits
  the target accurately the raw rate *is* the set-point, both filter
  settings converge on it, and the comparison is decided by noise -- which
  is how it failed on the ubuntu-clang runner.
  The two remaining tests pin the filter's weighting without depending on
  the rate actually achieved: with a0 == 1 the estimate must ignore the
  measurement entirely, and with a0 ~ 0 it must equal it. A comment records
  why the third test is deliberately absent.
* docs(mrpt_system): correct CControlledRateTimer's documented filter and defaults
  The header described the rate estimator's low-pass filter as
  `estimation = a0*input + (1-a0)*former_estimation`, but the
  implementation applies those weights the other way round, so the meaning
  of `a0` was inverted in the docs: larger values smooth *more*, not less.
  Two documented default values were also stale: `Ti` is 0.1 (documented
  as 0.0194) and `a0` is 0.99 (documented as 0.9).
  Only the documentation changed. The control law is the sensible reading
  for a low-pass filter and the PI gains were presumably tuned against it.
  A new CControlledRateTimer_unittest.cpp pins both halves so they cannot
  drift apart again: it asserts the documented defaults, and asserts that
  with a0 == 1 the estimate ignores the measurement entirely and stays on
  the set-point (under the previously-documented formula it would instead
  equal the raw measured rate). Parameter validation and setRate() are
  covered too.
* test(graphslam,system,io,slam): cover the largest untested pure-logic files
  Second coverage pass, on the four biggest pure-logic gaps left after
  mrpt_nav/mrpt_kinematics:
  mrpt_graphslam  41.0% -> 72.5% lines (33.0% -> 59.6% branches)
  mrpt_slam       64.1% -> 67.5%       (43.1% -> 44.9%)
  mrpt_io         57.0% -> 63.6%       (40.2% -> 46.4%)
  mrpt_system     54.4% -> 61.7%       (40.5% -> 41.9%)
  Five files that had never had a single assertion run against them are now
  covered: TSlidingWindow.cpp, CEdgeCounter.cpp, md5.cpp,
  CRejectionSamplingRangeOnlyLocalization.cpp (all 0%) and
  vector_loadsave.cpp (16%).
  Bugs found and fixed, each reproduced by a failing test first:
  * TSlidingWindow::getMean() divided by zero on an empty window and
  returned NaN. Since every comparison against a NaN is false,
  evaluateMeasurementAbove() was then stuck at false for any input.
  * TSlidingWindow::getStdDev() normalized by the window capacity instead
  of the number of measurements held, so a partially-filled window
  systematically under-reported sigma -- and disagreed with getMean(),
  which uses the sample count.
  * TSlidingWindow::resizeWindow() invalidated the mean and median caches
  but never the std-dev one: stale after a shrink, and on a grow it
  invalidated nothing at all even though the value depends on the window
  size.
  * CEdgeCounter::clearAllEdges() reset every counter except
  m_unique_edges, so a cleared instance still reported a stale
  unique-edge total.
  * mrpt::system::md5(const std::vector<uint8_t>&) used &str[0], an
  out-of-bounds access for an empty vector whose resulting pointer then
  tripped the ASSERT\_(data) of the overload it delegates to: md5() of an
  empty vector threw, while md5() of an empty string returned the
  correct digest. The algorithm itself passes every RFC 1321 vector.
  * mrpt::io::vectorNumericFromTextFile() had three defects: it discarded
  fscanf()'s return value in its default byRows==false path (the
  (!byRows) || short-circuits), so a failed read still pushed the stale
  value and an empty file yielded {0.0}; it never cleared its output
  vector, unlike loadTextFile()/loadBinaryFile(); and it leaked the FILE
  handle that every sibling function in the same file closes.
  CRejectionSamplingRangeOnlyLocalization was the largest 0% file in the
  repo but its 10 new tests all passed first time: it was untested, not
  broken.
  agents.md records the new numbers, the two behaviors left documented
  rather than changed, and a gotcha about the reproduce recipe: it excludes
  apps/, so mrpt_libapps_cli's ~40 CLI tests silently skip and the module
  measures ~8% instead of ~59%.
* Contributors: Jose Luis Blanco-Claraco

3.1.3 (2026-08-12)
------------------
* perf(mrpt_system): minimize CTimeLogger enter()/leave() overhead.
* fix(mrpt_system): make CTimeLogger reporting thread-safe vs. concurrent logging.
* fix(mrpt_system): fix GetTempPathA/GetTempFileNameA return value checking and duplicate temp paths on Windows.
* Contributors: Jose Luis Blanco-Claraco

3.1.2 (2026-07-07)
------------------

3.1.1 (2026-07-04)
------------------

3.1.0 (2026-07-03)
------------------
* Merge pull request `#1376 <https://github.com/MRPT/mrpt/issues/1376>`_ from MRPT/port/2x-fixes-jul2026
  Port 2.x fixes: COutputLogger thread-safety, octomap build flag
* Contributors: Jose Luis Blanco-Claraco

3.0.4 (2026-06-17)
------------------

3.0.3 (2026-06-15)
------------------
* Fix rawlog-edit CLI unittest failure on Windows CI (`#1370 <https://github.com/MRPT/mrpt/issues/1370>`_)
* Increase unit test coverage for mrpt_math, mrpt_io, mrpt_system (`#1365 <https://github.com/MRPT/mrpt/issues/1365>`_)
* Contributors: Jose Luis Blanco-Claraco

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

