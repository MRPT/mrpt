# MRPT 3.x AI Agent Instructions

Architecture context, conventions and known pitfalls for the Mobile Robot
Programming Toolkit (MRPT) 3.x. Follow these rules when generating, refactoring
or reviewing code in this repository.

> **Keeping this file short (read before editing it):**
> * Record only durable facts: conventions, commands, non-obvious assumptions,
>   recurring pitfalls. One or two lines per item.
> * Do NOT add change logs, dated entries, "pass history", before/after
>   numbers, lists of fixed bugs, or the story of why something changed. That
>   belongs in git commit messages.
> * Do NOT list APIs or classes: the headers and doxygen docs are the reference.
> * When updating, replace or delete stale text instead of appending.

## 1. Architecture and build

* Highly modular: each `modules/mrpt_*` directory is an independent CMake
  project, built with **colcon** (like ROS 2 packages). Cross-platform (Linux,
  Windows, macOS, WebAssembly/Emscripten).
* Build: `colcon build --packages-up-to mrpt_XXX`, then `. install/setup.bash`
  to run executables or tests. `colcon_defaults.yaml` at the repo root already
  sets symlink install, RelWithDebInfo and `CMAKE_EXPORT_COMPILE_COMMANDS`.
* CMake: `cmake_minimum_required(VERSION 3.16)`, always
  `find_package(mrpt_common REQUIRED)`, and define libraries with
  `mrpt_add_library` (handles C++ standard, exports, install, .so versioning):

```cmake
mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SOURCES} ${LIB_PUBLIC_HEADERS}
  PUBLIC_LINK_LIBRARIES mrpt::another_module
  CMAKE_DEPENDENCIES another_module
)
```

* Find other modules with `find_package(mrpt_<module> REQUIRED)`; link to
  namespaced targets (`mrpt::mrpt_poses`, `Eigen3::Eigen`).
* Do not hardcode compiler flags (`-std=c++17`, `-fPIC`), set .so versions or
  write manual `install()` blocks for standard headers/libraries.
* On the ROS build farm (both `ROS_DISTRO` and `JENKINS_URL` set),
  `mrpt_cmake_functions.cmake` forces `BUILD_TESTING=OFF`, since the tests
  already run in GitHub Actions CI. Override with
  `-DMRPT_FORCE_TESTS_ON_ROS_BUILDFARM=ON`.

## 2. C++ guidelines

* Modern C++ (C++17/20). All code inside `mrpt::` or its sub-namespaces.
* Formatting per `.clang-format` / `.clang-tidy`: braces always
  (`if (x) {\n y;\n }`, never `if (x) y;`); use `[[nodiscard]]` where applicable.
* No raw owning pointers: `std::shared_ptr` / `std::unique_ptr` and MRPT's
  smart pointer macros.
* Do not expose Eigen headers in public API headers unless the user allows it;
  keep Eigen `#include`s in `src/`.
* Avoid huge inline members: large stack objects broke exception backtraces
  on aarch64 (Ubuntu GCC 13, stack-clash protection).
* Prefer `std::optional` return values over bool + output-parameter APIs
  (keep the old signature as a `[[deprecated]]` inline shim when replacing one).
* Every new `.cpp`, `.h` and `CMakeLists.txt` starts with the MRPT header:

```cpp
/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
```

## 3. Unit tests

* Test files live in each module's `tests/` and end in `_unittest.cpp`. Many
  modules (`mrpt_maps`, `mrpt_nav`, `mrpt_math`, `mrpt_img`, ...) list them
  explicitly in `LIB_UNIT_TEST_SOURCES`: a new file never runs until registered
  in the module's `CMakeLists.txt`.
* Locate test data only via `mrpt::mrpt_data_dir()`, and use `GTEST_SKIP()`
  when it is missing (never print-and-return: that reports OK while testing
  nothing).
* GUI window tests must use `SKIP_IF_NO_GUI()` (`tests/gui_test_common.h`) and
  run under `xvfb-run`; `MRPT_SKIP_GUI_TESTS=1` forces the skip. Never
  pixel-compare window screenshots (readback is black without a compositor);
  just assert a frame of plausible size was grabbed.
* Render reference images (`mrpt_opengl` offscreen EGL/FBO tests) are
  regenerated with `MRPT_UPDATE_RENDER_REFERENCES=1
  build/mrpt_opengl/bin/test_mrpt_opengl` under `xvfb-run`. Their whole-frame
  tolerance is loose and a reference captured with a bug keeps passing: prefer
  asserting invariants on pixels (see `CFBORender_ScreenSpace_unittest.cpp`) or
  on CPU-side vertex buffers (`mrpt_viz/tests/RenderBuffers_unittest.cpp`).
* Useful test helpers:
  * `tests/legacy_serialization.h` (in `mrpt_math`, `mrpt_obs`, `mrpt_viz`,
    `mrpt_img`; duplicated because modules are independent) writes an object
    frame with an arbitrary serialization version, to reach the
    backwards-compatible branches of `serializeFrom()` without fixture files.
    The viz copy adds `writeLegacyRenderHeader()`.
  * `mrpt_hwdrivers/tests/mock_stream.h`: a scripted `CStream` for any driver
    using `bindIO()` / `bindStream()`. `mrpt_comms` tests use a local
    `CServerTCPSocket` (`comms_test_server.{h,cpp}`) and a pseudo-terminal for
    `CSerialPort`.
  * `mrpt::cpu::overrideDetectedFeature()` disables a SIMD feature so the
    vectorized and portable paths can be compared in one test.

## 4. Module-specific pitfalls

* **mrpt_math**: fixed-size matrices are explicitly instantiated only for some
  sizes (square `CMatrixFixed`: 2,3,4,6,7,12; `CVectorFixed`: 2,3,4,5,6,7,12;
  see `src/MatrixVectorBase_instantiate_*.cpp`). Other sizes compile but fail
  to link. Never `#pragma pack(1)` a struct holding a class like `TPoint3D`
  (SIGBUS on armhf).
* **mrpt_img**: `CImage::at<T>()` is a raw `reinterpret_cast`; for 3-channel
  images use `at<uint8_t>(x, y, channel)`, not `at<TColor>()`.
  `scaleHalf()`/`grayscale()` dispatch to SSE2/SSSE3 kernels and return whether
  a fast path ran; the gray kernel needs `width % 16 == 0`, and no kernel runs
  in place. `scaleHalf()` with `IMG_INTERP_NN` point-samples.
  `BayerPattern` names the top-left 2x2 block (ROS convention); OpenCV's
  `COLOR_Bayer*` names are shifted (ROS `RGGB` == `COLOR_BayerBG2RGB`).
* **mrpt_viz** has no OpenGL dependency (scene-graph description consumed by
  `mrpt_opengl`), so it is testable with plain unit tests.
* **mrpt_gui**: `mrpt/gui/WxUtils.h` pulls in wxWidgets headers but the library
  links wxWidgets privately; test targets need
  `target_link_libraries(... PRIVATE imp_wxwidgets)`. macOS/Windows CI builds
  with `-DDISABLE_WXWIDGETS=ON`.
* **mrpt_hwdrivers**: several `MRPT_HAS_*` macros (`MRPT_HAS_OPENCV`,
  `MRPT_HAS_LIBDC1394_2`, `MRPT_HAS_ROBOPEAK_LIDAR`, `MRPT_HAS_NIDAQMX*`,
  `MRPT_HAS_PGR_FLYCAPTURE2`, `MRPT_HAS_KINECT_CL_NUI`) are never defined in
  3.x, so those paths are compiled out; defining one alone does not make it
  build. Some sources never include `mrpt/hwdrivers/config.h`, so even defined
  macros read 0 there.
* **mrpt_nav**: tests must advance the *navigation* time
  (`getNavigationTime()`), not just the clock, due to a 20 ms throttle in
  `updateCurrentPoseAndSpeeds()`. `CPTG_DiffDrive_*` need a polygonal
  `shape_x0`/`shape_y0`/... config; `CPTG_Holo_Blend` takes `robot_radius`
  (rejects polygons), and a waypoint with default `speed_ratio = 1.0` yields no
  viable movement.
* **mrpt_obs**: `CObservationVelodyneScan` per-ray timestamps derive from
  `CObservation::timestamp`, not `getOriginalReceivedTimeStamp()`.
* **mrpt_maps**: `CVoxelMapRGB` / `CColouredOctoMap` color 3D scans via
  `hasRangeImage` + camera intrinsics, not `hasPoints3D` (a hand-built
  `CObservation3DRangeScan` needs `setIntrinsicParamsFromValues()` and a filled
  `rangeImage`). `CGasConcentrationGridMap2D::build_Gaussian_Wind_Grid()`
  caches a LUT file in the current working directory.
  `COccupancyGridMap3D`'s `determineMatching2D()`, `compute3DMatchingRatio()`
  and `internal_computeObservationLikelihood()` are unimplemented and throw.
* **mrpt_slam**: only the auxiliary particle filters go through
  `PF_SLAM_implementation_gatherActionsCheckBothActObs()`; `pfStandardProposal`
  reads the action directly. An empty sensory frame counts as valid: pass a
  null `sf` to keep a movement accumulated.
* **mrpt_topography**: every ENU helper uses the ellipsoid normal as "Up".
  The inverse of `geodeticToENU_WGS84()` is `ENUToGeodetic_WGS84()` (returns
  lon/lat/height); `ENUToGeocentric()` returns geocentric (ECEF) coordinates
  and inverts `geocentricToENU_WGS84()`. In MRPT 2.x and 3.0.x,
  `ENUToGeocentric()` used a geocentric-radial "Up" instead.
* Known-unreachable code kept on purpose: `mrpt_rtti`'s deferred
  class-registration queue, the `shared_ptr<yaml>` alternative in
  `mrpt_containers`' `scalar_t` (ABI-affecting), and `mrpt_viz`'s
  `CTextMessageCapable::regenerateGLobjects()`.

## 5. Recurring defect shapes worth grepping for

* `if (version >= N)` with no `else` in `serializeFrom()`: fields missing from
  an old stream keep the reused object's previous values.
* `for (size_t i = 0; i < n - 1; i++)` underflows when `n == 0`; write
  `i + 1 < n`.
* A method taking a non-recursive lock and calling another one that locks the
  same mutex.
* Output parameters read instead of written, output containers taken by value
  or not resized, a callee that overwrites (rather than appends to) an output
  the caller had already filled, and "in-place" operations that free the
  source buffer before reading it.
* Members read but never written; declared-but-never-defined functions.
* Documented defaults set only in `loadFromConfigFile()`, with the member
  itself left uninitialized.
* A getter/setter pair sharing a name where the setter's only argument has a
  default: the no-argument call on a non-const object resolves to the setter.
* Doxygen that references functions that no longer exist in 3.x.

## 6. Python bindings (pybind11)

* Each module `mrpt_foo` has its own Python module, made of:
  1. `modules/mrpt_foo/python_bindings/mrpt_foo_py.cpp` defining
     `PYBIND11_MODULE(_bindings, m)` (the library is always `_bindings.so`).
  2. `modules/mrpt_foo/python/mrpt/foo/__init__.py`: `from . import _bindings
     as _b`, `ClassName = _b.ClassName` per wrapped name, and `__all__`.
     Pythonic extras (`__array__`, operators, constants) may be monkey-patched
     here.
  3. `mrpt_add_python_module(foo python_bindings/${PROJECT_NAME}_py.cpp)` in
     the module's `CMakeLists.txt` (uncomment it if commented out).
* `mrpt` is a PEP 420 namespace package: never add an `mrpt/__init__.py`.
* `package.xml` must `build_depend` on `pybind11-dev` and `python3-dev`:
  `mrpt_add_python_module()` silently skips the bindings if pybind11 is not
  found (e.g. on the ROS build farm).
* A new Python module also needs its own `python3-mrpt-<mod>` Debian package
  (`debian/control`, the `python3-mrpt` metapackage, a `.install` file and the
  import list in `debian/tests/control`); see "Releases" below.
* Examples live in `mrpt_examples_py`; extend them when wrapping new classes.
* Conventions:
  * Include `<pybind11/pybind11.h>` and `<pybind11/stl.h>`; add `eigen.h`,
    `numpy.h`, `operators.h`, `chrono.h`, `functional.h` as needed.
  * `py::class_<Derived, Base..., std::shared_ptr<Derived>>`, including
    `CSerializable` in the bases of serializable classes.
  * MRPT `x()` / `x(double)` accessor pairs become `def_property` with
    lambdas; public members use `def_readwrite` / `def_readonly`.
  * Resolve overloads with `py::overload_cast<...>`; turn output-argument
    methods into lambdas returning a value or `py::make_tuple(...)`.
  * Always define `__str__` and `__repr__`.
  * NumPy: Eigen types convert automatically (zero-copy via
    `Eigen::Map<const RowMajorMatrix>`); `CImage` exposes a
    `py::array_t<uint8_t>` with the Python object as buffer base.
  * Iterators: `py::make_iterator(...)` with `py::keep_alive<0, 1>()`.
    Enums: `py::enum_` + `export_values()`, `py::arithmetic()` for bitmasks.
  * Acquire the GIL (`py::gil_scoped_acquire`) in C++ callbacks invoked from
    other threads.
  * `reference_internal` for getters returning internal references,
    `reference` for singletons.

## 7. Code coverage

Goal: 90% line coverage per module. Branch coverage lags lines by 15-30 points
nearly everywhere: prioritize failure-path tests.

```bash
find build -iname '*.gcda' -delete   # stale profiles corrupt the numbers
colcon build --base-paths modules apps --cmake-args -DENABLE_COVERAGE=ON -DBUILD_TESTING=ON
xvfb-run -a --server-args="-screen 0 1280x1024x24" colcon test --base-paths modules apps
gcovr --root . -j$(nproc) --gcov-executable gcov-$(gcc -dumpversion | cut -d. -f1) \
  --gcov-ignore-parse-errors=all --merge-mode-functions=merge-use-line-min \
  --exclude-unreachable-branches --exclude-throw-branches \
  --exclude '.*/3rdparty/.*' --exclude '.*/stb/.*' --exclude '.*/tests/.*' \
  --exclude '.*_unittest\.cpp' --exclude '.*/python_bindings/.*' --exclude '.*/samples/.*' \
  --json-pretty -o coverage.json build
scripts/coverage_module_report.py coverage.json mrpt_math   # per-file + aggregate
```

* `xvfb-run`: otherwise GUI tests skip. `apps` in `--base-paths`: otherwise
  `rawlog-edit` CLI tests skip.
* `gcov` major version must match the compiler (use `llvm-cov gcov` for
  clang). The two `gcovr` flags after it avoid aborts on large files and on
  header templates reported at different lines.
* Always use `scripts/coverage_module_report.py`: with symlink install every
  header is reported twice (`modules/` and `install/`), and raw gcovr totals
  are wrong without merging.
* Measure after rebuilding and testing *all* modules: template-heavy headers
  get most of their coverage from other modules' tests.
* Main remaining gaps: `mrpt_hwdrivers` drivers that own their transport
  (need `bindIO()`/`bindStream()` first), `mrpt_imgui` and non-wx parts of
  `mrpt_gui`, and `mrpt_libapps_cli` (better covered by subprocess tests).

## 8. Porting ROS 2 nodes

See "Porting ROS 2 nodes" in `doc/source/doxygen-docs/port_mrpt3.md`:
`package.xml` renames (`mrpt_lib*` to `mrpt_<module>`), CMake target renames
(`mrpt::<X>` to `mrpt::mrpt_<X>`), and replacing `mrpt_libtclap` with CLI11.

## 9. Releases

* Full procedure: `doc/source/make_a_mrpt_release.rst`. Each module/app has its
  own `package.xml` version and `CHANGELOG.rst`, bumped with
  `catkin_prepare_release` on `develop`; then merge into `master`, tag, and
  package with `packaging/make_release.sh`.
* `packaging/release.py` automates the whole flow (use `--dry-run` first).
* Never run `release.py` or any step that pushes, tags or publishes unless the
  user explicitly asks for a release.
* Debian/Ubuntu packaging lives outside this repo, in two trees that must be
  kept consistent: the official Debian one (salsa.debian.org
  `robotics-team/mrpt`, `master`) and the Ubuntu PPA one
  (github.com/MRPT/mrpt-ubuntu-ppa-packages, one `debian/`-only branch per
  distro: `noble`, `resolute`). Whenever a module, library or Python package
  is added, removed or renamed, or the `MAJOR.MINOR` SOVERSION changes, update
  `debian/` in both trees and all PPA distro branches.

## 10. Agent tool usage

* Use `git -C foo ...` instead of `cd foo && git ...`.
