# MRPT 3.x AI Agent Instructions

Welcome, AI Agent! This file contains the architectural context and coding guidelines for the Mobile Robot Programming Toolkit (MRPT) 3.0. When generating, refactoring, or reviewing code in this repository, you must strictly adhere to the following rules.

## 1. Project Architecture
* **MRPT 3.0 is highly modular.** It is designed to be built using **colcon** (similar to ROS 2 packages).
* Each module (e.g., `mrpt_opengl`, `mrpt_math`) lives in its own directory and functions as an independent CMake project.
* **Target OS/Compilers:** Cross-platform (Linux, Windows, macOS, WebAssembly/Emscripten).

## 2. Build System (CMake) Conventions
MRPT 3.0 abstracts away standard CMake boilerplate using `mrpt_common`. Use standard CMake commands but prefer mrpt_common cmake helpers when possible for consistency.

* **Compiling a module**: Use ``colcon build --packages-up-to mrpt_XXX``, then ``. install/setup.bash`` then you can run the executables or unit tests.
  There is already a `colcon_defaults.yaml` at the repo root defining symlink install, RelWithDebInfo builds, and `CMAKE_EXPORT_COMPILE_COMMANDS`.

* **Minimum CMake Version:** `cmake_minimum_required(VERSION 3.16)`
* **Always include the MRPT common scripts:**

```cmake
find_package(mrpt_common REQUIRED)
```

* Target Definition: Use mrpt_add_library instead of add_library. This macro automatically handles C++ standard configurations, export targets, and installation steps.

```cmake
mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SOURCES} ${LIB_PUBLIC_HEADERS}
  PUBLIC_LINK_LIBRARIES mrpt::another_module
  CMAKE_DEPENDENCIES another_module
)
```

Dependency Management: 

* Use find_package(mrpt_<module_name> REQUIRED) to find other MRPT modules.

Link against namespaced targets (e.g., mrpt::mrpt_poses, Eigen3::Eigen).

## 3. C++ Coding Guidelines
Standard: Use Modern C++ features where appropriate (C++17/C++20).

Namespaces: All core code must reside within the mrpt:: namespace or its sub-namespaces (e.g., mrpt::opengl::).

Unit tests: test files are in a "tests" subdirectory in each module, and their name must end in `_unittest.cpp`, then they will be catched automatically for inclusion as unit tests.

License Headers: Every new .cpp, .h, and CMakeLists.txt file must start with the standard MRPT SPDX-License-Identifier header:

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

Formatting: according to `.clang-format` and `.clang-tidy`. In particular: prefer `if (x) {\n y;\n }` instead of `if(x) y;`. Use `[[nodiscard]]` where applicable.

## 4. Anti-Patterns to Avoid
Do not manually configure .so versioning or write manual install() blocks for standard headers/libraries. mrpt_add_library does this.

Do not hardcode compiler flags (like -std=c++17 or -fPIC). The MRPT CMake wrappers handle these natively.

Do not use raw pointers for ownership. Default to std::shared_ptr or std::unique_ptr, and use MRPT's smart pointer macros where applicable.

Do not expose Eigen3 headers in public API files unless explicitly allowed by the user. Keep Eigen3 specific #include's in private "src/" files.

## 5. Pybind11 modules

In mrpt 3.x, most MRPT libraries under `ROOT/modules/mrpt_*` now have their own pybind11 module, in a modular way so users can import
only the required modules.

* `modules/mrpt_core/python/mrpt/__init__.py` is the main root python file for all modules, in charge of trying to import the rest, if they exist.
* Each module, for example `mrpt_img`, has its own `modules/mrpt_img/python/mrpt/img/__init__.py` that must be updated with new wrapped C++ classes.
* Each module, for example `mrpt_img`, has its own `modules/mrpt_img/python_bindings/mrpt_img_py.cpp` file with the specific wrapped C++ classes and python adaptors.
* Python examples, demonstrating each module wrapped features, live under `mrpt_examples_py`. They should be updated/extended or new examples created when appropriate as new classes are wrapped.

### 5.1 File structure for each pybind11 module

To add or extend Python bindings for a module `mrpt_foo`, three files are needed:

1. **`modules/mrpt_foo/python_bindings/mrpt_foo_py.cpp`** — The C++ pybind11 source. Must define `PYBIND11_MODULE(_bindings, m) { ... }`. The compiled shared library is always named `_bindings.so`.
2. **`modules/mrpt_foo/python/mrpt/foo/__init__.py`** — Python re-exports: `from . import _bindings as _b`, then `ClassName = _b.ClassName` for each wrapped class/function, and an `__all__` list.
3. **`modules/mrpt_foo/CMakeLists.txt`** — Must call `mrpt_add_python_module(foo python_bindings/${PROJECT_NAME}_py.cpp)`. If this line is commented out, uncomment it.

The root `mrpt/__init__.py` (in `mrpt_core`) auto-imports all known submodules by name. If adding a new module name, add it to the `MRPT_MODULES` list there.

### 5.2 Pybind11 coding patterns and conventions

**Includes:** Always include `<pybind11/pybind11.h>` and `<pybind11/stl.h>`. Add `<pybind11/eigen.h>` for Eigen/matrix types, `<pybind11/numpy.h>` for `py::array_t<T>`, `<pybind11/operators.h>` for operator overloading, `<pybind11/chrono.h>` for time types, `<pybind11/functional.h>` for `std::function` callbacks.

**Class binding with inheritance:**
```cpp
py::class_<Derived, Base1, Base2, std::shared_ptr<Derived>>(m, "Derived")
```
Always use `std::shared_ptr<T>` holder for classes that are commonly used via smart pointers. Include `CSerializable` in the base list for serializable classes.

**Properties:** When C++ uses `x()` getter and `x(double)` setter (common in MRPT), use:
```cpp
.def_property("x",
    [](const T& p) { return p.x(); },
    [](T& p, double val) { p.x(val); })
```
For public member variables, use `.def_readwrite("name", &T::name)` or `.def_readonly(...)`.

**Overloaded methods:** Resolve with `py::overload_cast<ArgTypes...>(&Class::method)` or `static_cast<RetType(Class::*)(ArgTypes...)>(&Class::method)`.

**Output-argument functions → return values:** Wrap in lambda:
```cpp
.def("compute", [](const T& self) {
    ResultType result;
    bool ok = self.compute(result);
    return py::make_tuple(ok, result);
})
```

**Operator overloading:** Use `py::self + py::self`, etc. Always add `__str__` and `__repr__`.

**NumPy integration:**
- For Eigen matrices: `#include <pybind11/eigen.h>` enables automatic conversion. For zero-copy, return `Eigen::Map<const RowMajorMatrix>(ptr, rows, cols)`.
- For `CImage`: Use `py::array_t<uint8_t>` with shape/strides and pass `self_obj` (the Python object) as the buffer base for zero-copy.
- Add `__array__` protocol: `.def("__array__", [](const T& self) { return self.as_numpy(); })`.

**Iterators:** `py::make_iterator(container.begin(), container.end())` with `py::keep_alive<0, 1>()`.

**Enums:** `py::enum_<T>(m, "Name").value("A", T::A).export_values();` Use `py::arithmetic()` for bitmask enums.

**GIL management:** When passing Python callables to C++ threads, acquire the GIL: `py::gil_scoped_acquire gil;` inside the C++ callback lambda.

**Return value policies:** Use `py::return_value_policy::reference_internal` for getters returning references to internal objects (e.g., `getCamera()` on a Viewport). Use `py::return_value_policy::reference` for singletons/static data.

**Python `__init__.py` conventions:**
- Re-export all bound classes: `ClassName = _b.ClassName`
- Monkey-patch `__array__` for NumPy integration where appropriate
- Add Pythonic conveniences (e.g., `<<` operator for scene building, color constants)
- Include `__all__` listing all public names

## 6. Porting ROS 2 nodes

For ROS 2 packages that previously depended on `mrpt_ros` (MRPT 2.x wrappers), see the
"Porting ROS 2 nodes" section of the porting guide at
`doc/source/doxygen-docs/port_mrpt3.md` for:

- `package.xml` dependency renaming (`mrpt_lib*` → fine-grained `mrpt_<module>` packages)
- `CMakeLists.txt` `find_package` and target renaming (`mrpt::<X>` → `mrpt::mrpt_<X>`)
- Migrating away from the dropped `mrpt_libtclap` / `mrpt-tclap` to CLI11

## 7. Instructions for AI agents

- Do not use the compound command "cd foo && git ...", instead, use "git -C foo ...".

## 8. Making a release

Full procedure: `doc/source/make_a_mrpt_release.rst`. Quick summary:

* Each module/app has its own `package.xml` version and `CHANGELOG.rst`,
  bumped together via `catkin_prepare_release` (from the `catkin_pkg`
  Python package), run from the repo root on branch `develop`.
* `develop` is then merged into `master`, tagged, and packaged with
  `packaging/make_release.sh` (produces signed `.tar.gz`/`.zip` in
  `$HOME/mrpt_release/`).
* `packaging/release.py` automates the whole flow end-to-end (version bump
  → changelog → merge to master → tarball + GPG signature → `gh release
  create` with the tarball/zip/signature attached for Debian's `uscan`),
  pausing for explicit confirmation before pushing `master` or publishing
  the GitHub release. Run with `--dry-run` first to preview the commands.
* Do not run `packaging/release.py` or any step that pushes/tags/publishes
  unless the user explicitly asks for an actual release to be cut.

## 9. ROS build farm "dev" jobs skip tests (2026-07-10)

ROS "*dev" buildfarm Jenkins jobs (e.g. `Kdev__mrpt3__ubuntu_noble_amd64`) run
`colcon build -DBUILD_TESTING=0` once, then a **second, fully clean**
`colcon build --cmake-clean-cache -DBUILD_TESTING=1` pass that also compiles
every module's gtest binaries. This doubles build time and was observed to
blow the 120-minute Jenkins timeout mid-way through the second pass, before
`colcon test` was ever invoked (no test results were produced at all). The
same test suite already runs on every push/PR via GitHub Actions CI
(`.github/workflows/build-linux.yml`), which never sources a ROS environment.

Fix: `modules/mrpt_common/cmake/mrpt_cmake_functions.cmake` now forces
`BUILD_TESTING` back to `OFF` (overriding the buildfarm's explicit
`-DBUILD_TESTING=1`) whenever both `ROS_DISTRO` (sourced ROS env) and
`JENKINS_URL` (any Jenkins job) are set in the environment — i.e. only on the
ROS build farm itself, not for a developer who merely has ROS sourced
locally. Escape hatch: `-DMRPT_FORCE_TESTS_ON_ROS_BUILDFARM=ON` re-enables
tests there if ever needed. Since both `mrpt_add_test()` and
`mrpt_add_python_binding_test()` already gate on `BUILD_TESTING`, no other
files needed changes.

## 10. Code Coverage

**Goal: 90% line coverage per module.** Overall (2026-09-09, deduplicated as
`scripts/coverage_module_report.py` does): **81.2% lines / 59.9% branches**. The remaining gap is
dominated by the hardware/GUI modules.

### Reproducing the numbers

```bash
colcon build --base-paths modules apps --cmake-args -DENABLE_COVERAGE=ON -DBUILD_TESTING=ON
xvfb-run -a --server-args="-screen 0 1280x1024x24" colcon test --base-paths modules apps
gcovr --root . -j$(nproc) --gcov-executable gcov-13 \
  --gcov-ignore-parse-errors=all --merge-mode-functions=merge-use-line-min \
  --exclude-unreachable-branches --exclude-throw-branches \
  --exclude '.*/3rdparty/.*' --exclude '.*/stb/.*' --exclude '.*/tests/.*' \
  --exclude '.*_unittest\.cpp' --exclude '.*/python_bindings/.*' --exclude '.*/samples/.*' \
  --json-pretty -o coverage.json build
scripts/coverage_module_report.py coverage.json mrpt_math   # per-file + aggregate
```

Every flag above is there because something breaks without it:

* **`xvfb-run`**: without a display the `mrpt_gui` window tests `GTEST_SKIP()`
  and the module reads ~1% instead of ~40%. `MRPT_SKIP_GUI_TESTS=1` forces the
  skip.
* **`--gcov-executable gcov-<compiler major version>`**: the system `gcov`
  alias may point at an unrelated binary, and gcovr then silently mis-decodes
  its output.
* **`--gcov-ignore-parse-errors=all`**: large files trip gcovr's "suspicious
  hits" detector, which otherwise aborts the whole run.
* **`--merge-mode-functions=merge-use-line-min`**: gcovr >= 8.6 aborts when a
  header-only template function is reported at two different line numbers by
  different `.gcda` files.
* **`apps` in `--base-paths`**: without it `rawlog-edit` is never built and the
  ~40 `RawlogEditCLITest` cases all skip, reading `mrpt_libapps_cli` at ~8%.
* **`scripts/coverage_module_report.py`**: with symlink-install every header is
  reported twice (under `modules/` and under `install/`). The script merges
  duplicates (max hit-count per line, OR-merged branches) before computing
  percentages; raw gcovr CLI output is wrong in both directions without it.
* Delete stale `.gcda` (`find build -iname '*.gcda' -delete`) before a
  measuring run: re-running an instrumented binary outside `colcon test`
  leaves mismatched-checksum profiles behind.

**Measuring one module after changing it**: you must still rebuild and retest
*everything*. Template-heavy public headers (`CMatrixFixed.h`, `TPoint3D.h`, …)
get most of their instantiation coverage from *other* modules' tests, so an
isolated single-package run can read 30-40 points low. Incremental
`colcon build`/`colcon test` with no `--packages-*` filter is a few minutes.

### Coverage by module (worst first)

| Module | Covered/Total lines | Line % | Branch % |
|---|---|---|---|
| mrpt_imgui | 0/53 | 0.0% | 0.0% |
| mrpt_hwdrivers | 1966/6747 | 29.1% | 20.7% |
| mrpt_gui | 1888/4655 | 40.6% | 30.4% |
| mrpt_opengl | 2322/4234 | 54.8% | 35.7% |
| mrpt_libapps_cli | 1129/1910 | 59.1% | 38.0% |
| mrpt_libapps_gui | 803/1288 | 62.3% | 45.8% |
| mrpt_common | 5/7 | 71.4% | n/a |
| mrpt_comms | 694/906 | 76.6% | 54.9% |
| mrpt_graphslam | 814/997 | 81.6% | 64.4% |
| mrpt_system | 1639/1964 | 83.5% | 59.9% |
| mrpt_rtti | 151/176 | 85.8% | 78.4% |
| mrpt_io | 1133/1310 | 86.5% | 71.0% |
| mrpt_viz | 8694/9857 | 88.2% | 68.5% |
| mrpt_maps | 10637/11790 | 90.2% | 66.4% |
| mrpt_nav | 5670/6276 | 90.3% | 69.0% |
| mrpt_containers | 1809/1999 | 90.5% | 56.0% |
| mrpt_core | 579/638 | 90.8% | 71.8% |
| mrpt_obs | 6287/6857 | 91.7% | 63.2% |
| mrpt_graphs | 1030/1113 | 92.5% | 76.8% |
| mrpt_poses | 6282/6788 | 92.5% | 61.3% |
| mrpt_expr | 93/100 | 93.0% | 60.2% |
| mrpt_slam | 4062/4348 | 93.4% | 65.3% |
| mrpt_bayes | 1049/1090 | 96.2% | 80.3% |
| mrpt_serialization | 728/754 | 96.6% | 76.9% |
| mrpt_math | 7611/7864 | 96.8% | 65.6% |
| mrpt_random | 162/167 | 97.0% | 88.4% |
| mrpt_tfest | 635/654 | 97.1% | 72.9% |
| mrpt_kinematics | 503/518 | 97.1% | 81.2% |
| mrpt_config | 536/548 | 97.8% | 84.7% |
| mrpt_img | 3036/3100 | 97.9% | 77.2% |
| mrpt_topography | 414/417 | 99.3% | 83.2% |
| mrpt_typemeta | 57/57 | 100.0% | 80.9% |

### Techniques that work

* **Legacy serialization without fixture files.** A ~30-line helper writes an
  MRPT object frame (`[len|0x80][class name][version byte][payload][0x88]`)
  with an *arbitrary* streaming version, so the backwards-compatible branches
  of `serializeFrom()` can be driven directly. It lives as
  `tests/legacy_serialization.h` in `mrpt_math`, `mrpt_obs`, `mrpt_viz` and
  `mrpt_img` (duplicated, since modules are independent CMake projects). The
  viz copy adds `writeLegacyRenderHeader()`, because every viz class starts
  with `CVisualObject::writeToStreamRender()`, versioned independently of the
  class itself. This is consistently the highest-yield technique available.
* **Mock transports.** `mrpt_hwdrivers/tests/mock_stream.h` is a `CStream` that
  records writes and replays scripted answers keyed on the command received —
  enough for any driver reachable via `C2DRangeFinderAbstract::bindIO()` or
  `CGPSInterface::bindStream()`. `mrpt_comms` uses a one-shot local
  `CServerTCPSocket` (`comms_test_server.{h,cpp}`) for its HTTP/NTRIP client
  and a pseudo-terminal (`posix_openpt`) for `CSerialPort`, with no hardware
  and no network.
* **`mrpt::cpu::overrideDetectedFeature()`** forces a SIMD feature off, so both
  the vectorized and the portable path of the same function can be asserted
  against each other in one test regardless of the host CPU.
* **Headless GUI.** `xvfb-run` plus Mesa's software rasterizer is enough to
  open and drive `CDisplayWindow*`. Do **not** pixel-compare window
  screenshots: with no compositor the window is never mapped and readback is
  uniformly black even though rendering happened. Assert that a frame was
  grabbed with a plausible size; reference-image comparisons belong in
  `mrpt_opengl`'s offscreen EGL/FBO tests. `tests/gui_test_common.h` provides
  `SKIP_IF_NO_GUI()`, which every window test must use.
* **Regenerating render references**: `MRPT_UPDATE_RENDER_REFERENCES=1
  build/mrpt_opengl/bin/test_mrpt_opengl` under `xvfb-run`. Beware: a reference
  image captured while a defect was present will happily keep passing —
  `mrpt_viz/tests/RenderBuffers_unittest.cpp` asserts on the CPU-side vertex
  buffers instead, which is what actually caught several "renders nothing"
  regressions.

### Recurring defect shapes worth grepping for

Roughly 90 real bugs have been found by these passes. The ones that recur:

* `if (version >= N)` with no `else` in `serializeFrom()`: fields absent from
  an older stream keep the *reused* destination object's previous values
  instead of being reset to their defaults.
* Unsigned underflow in loop bounds: `for (size_t i = 0; i < n - 1; i++)` spins
  ~2^64 times when `n == 0`. Write `i + 1 < n`.
* A method that takes a non-recursive lock and then calls another method that
  locks the same mutex ("Resource deadlock avoided").
* Output parameters written into instead of read out of; output containers
  taken **by value**; output vectors written without being resized.
* Members read but never written; declared-but-never-defined functions (a link
  error for any caller, invisible while nothing calls them).
* Documented defaults that live only in `loadFromConfigFile()` while the member
  itself is left uninitialized.
* A getter/setter pair sharing a name where the setter's only argument is
  defaulted: the no-argument call resolves to the *setter* on a non-const
  object.
* `CImage::at<T>()` is a raw `reinterpret_cast`: `at<TColor>()` on a 3-channel
  image writes 4 bytes over a 3-byte pixel. Use `at<uint8_t>(x, y, channel)`.
* Branch coverage lags line coverage nearly everywhere by 15-30 points:
  error-handling and edge-case branches are what is left untested even in files
  with good line coverage. Prioritize failure-path tests over more happy paths.

### Per-module notes for future passes

* **Explicit `LIB_UNIT_TEST_SOURCES` lists** (not glob-based) in `mrpt_maps`,
  `mrpt_nav`, `mrpt_math`, `mrpt_img` and others: a new `*_unittest.cpp`
  silently never runs until it is registered in the module's `CMakeLists.txt`.
* **`mrpt_math`** only explicitly instantiates fixed-size matrices for a few
  dimensions (square `CMatrixFixed`: 2,3,4,6,7,12; `CVectorFixed`:
  2,3,4,5,6,7,12 — see `src/MatrixVectorBase_instantiate_*.cpp`). Instantiating
  a template with any other size compiles but fails to *link*. Pick 2 as the
  smallest.
* **`mrpt_hwdrivers`**: several `MRPT_HAS_*` macros the sources still guard on
  are never defined in the 3.x build (`MRPT_HAS_OPENCV`, `MRPT_HAS_LIBDC1394_2`,
  `MRPT_HAS_ROBOPEAK_LIDAR`, `MRPT_HAS_NIDAQMX*`, `MRPT_HAS_PGR_FLYCAPTURE2`,
  `MRPT_HAS_KINECT_CL_NUI`), so those paths are compiled out. Do not "fix" one
  by adding the define alone — `CImageGrabber_dc1394.cpp` no longer compiles
  against current `mrpt::img`. Several sources also never include
  `mrpt/hwdrivers/config.h`, so even their defined macros read 0.
* **`mrpt_nav`**: `rnav_unittest.cpp`'s helper returns silently when the shared
  `navigation-ptgs/*.ini` files are missing *and* swallows every exception, so
  it can pass while testing nothing — build configurations with
  `CConfigFileMemory` instead. Reactive tests must advance the robot's
  *navigation* time (`getNavigationTime()`), not just the clock, or
  `updateCurrentPoseAndSpeeds()`'s 20 ms throttle leaves the pose cache empty.
  `CPTG_DiffDrive_*` need a polygonal `shape_x0`/`shape_y0`/... in the config;
  `CPTG_Holo_Blend` takes `robot_radius` and rejects a polygonal shape, and
  with it a waypoint left at the default `speed_ratio = 1.0` yields no viable
  movement at all.
* **`mrpt_obs`**: `CObservationVelodyneScan`'s per-ray timestamps derive from
  `CObservation::timestamp`, not `getOriginalReceivedTimeStamp()`.
  `CObservationGPS`'s `TIMECONV_IsALeapYear()`/`GetNumberOfDaysInMonth()` are
  only reachable from a `seconds >= 60.0` rollover that floating-point drift
  never produces in practice.
* **`mrpt_maps`**: `CVoxelMapRGB`'s and `CColouredOctoMap`'s 3D-scan colour
  paths unproject via `hasRangeImage` + camera intrinsics, *not*
  `hasPoints3D` — a hand-built `CObservation3DRangeScan` needs
  `setIntrinsicParamsFromValues()` plus a filled `rangeImage`.
  `CGasConcentrationGridMap2D::build_Gaussian_Wind_Grid()` caches a LUT file in
  the *current working directory*, so a test covering both the "generate" and
  "load" branches must `chdir` into a scratch dir first.
  `COccupancyGridMap3D::determineMatching2D()`, `::compute3DMatchingRatio()`
  and `::internal_computeObservationLikelihood()` are unimplemented stubs that
  throw; tests assert the throwing behavior.
* **`mrpt_slam`**: only the *auxiliary* particle filters go through
  `PF_SLAM_implementation_gatherActionsCheckBothActObs()`;
  `pfStandardProposal` reads the action directly. An empty sensory frame still
  counts as "valid" — pass a null `sf` to leave a movement accumulated.
* **`mrpt_viz`** has zero OpenGL dependency (it is the scene-graph description
  consumed by `mrpt_opengl`), so almost all of it is testable with plain,
  non-rendering unit tests.
* **`mrpt_gui`**: `mrpt/gui/WxUtils.h` pulls in wxWidgets headers but the
  library links wxWidgets *privately*, so a test target needs an explicit
  `target_link_libraries(... PRIVATE imp_wxwidgets)`. The macOS/Windows CI jobs
  build with `-DDISABLE_WXWIDGETS=ON`, so window tests only run on Linux.

### Where the remaining gap is

1. **Hardware drivers — `mrpt_hwdrivers`**: what is still at 0% are the drivers
   that own their transport instead of reading through an injectable `CStream`
   (`COpenNI2Generic`, `CKinect`, `CCameraSensor`, `CNTRIPClient`, `CLMS100eth`,
   `CSICKTim561Eth`, `CCANBusReader`, `CTaoboticsIMU`, …). Reaching them needs
   the same `bindIO()`/`bindStream()` treatment first.
2. **GUI/rendering — `mrpt_imgui` (0%) and the rest of `mrpt_gui`**:
   `CDisplayWindowGUI.cpp` (nanogui/GLFW), `CQtGlCanvasBase.cpp` (Qt),
   `CImGuiSceneView.cpp`, the modal dialogs in `CAboutBox*`/`error_box.cpp`,
   and the rest of `mathplot.cpp`. The `xvfb-run` technique should work for the
   nanogui/Qt canvases too.
3. **CLI apps — `mrpt_libapps_cli`**: some `rawlog-edit_*.cpp` paths. Better
   suited to subprocess/golden-file integration tests than unit tests.
4. **Pure-logic files at 0%**: none left.
5. **Largest single-file gaps**:
   `mrpt_nav/src/reactive/CAbstractPTGBasedReactive.cpp`,
   `mrpt_maps/include/mrpt/maps/CVoxelMapOccupancyBase.h` (voxel types other
   than the two instantiated ones), `mrpt_maps/src/maps/CGenericPointsMap.cpp`,
   `mrpt_maps/src/maps/COccupancyGridMap2D_common.cpp`.
6. **Known-unreachable code, deliberately left in place**: `mrpt_rtti`'s
   deferred class-registration queue (dead since MRPT 1.x's `CLASS_INIT`;
   removing it would take the module to ~94%), the `shared_ptr<yaml>`
   alternative in `mrpt_containers`' `scalar_t` (ABI-affecting to remove), and
   `mrpt_viz`'s `CTextMessageCapable::regenerateGLobjects()` (`mrpt_opengl`
   builds text overlays directly from the label strings).

### Pass history

One line per pass; the details are in the git log.

| Date | Modules | Before -> after (lines) |
|---|---|---|
| 2026-07-03 | baseline for all 33 modules | — |
| 2026-07-06 | `mrpt_graphs`, `mrpt_random`, `mrpt_libapps_cli` | libapps_cli 9.1% -> 59.2% |
| 2026-07-07 | `mrpt_tfest` | -> 97.1% |
| 2026-07-09 | `mrpt_bayes`, `mrpt_config` | both -> >96% |
| 2026-07-10 | `mrpt_img`, `mrpt_obs`, `mrpt_topography` | img -> 93.3% |
| 2026-07-11 | `mrpt_containers` | -> 90.2% |
| 2026-07-17 | `mrpt_maps` (2 passes) | 48.1% -> 83.0% |
| 2026-08-02 | `mrpt_viz` | 29.5% -> 68.4% |
| 2026-08-03 | `mrpt_maps` (pass 3) | 83.0% -> 86.6% |
| 2026-08-28 | `mrpt_nav`, `mrpt_kinematics`; then `mrpt_graphslam`/`mrpt_system`/`mrpt_slam`/`mrpt_io` 0%-files | nav 63.0% -> 90.0%, kinematics 38.2% -> 96.8% |
| 2026-08-29 | `mrpt_serialization`, `mrpt_rtti`, `mrpt_comms`, `mrpt_io`, `mrpt_system` | comms 23.4% -> 75.8% |
| 2026-08-31 | `mrpt_slam` | 67.9% -> 90.2% |
| 2026-09-06 | `mrpt_graphslam`, `mrpt_gui` | gui 0.5% -> 40.6% |
| 2026-09-07 | `mrpt_math`/`mrpt_maps`/`mrpt_obs`/`mrpt_slam`; then `mrpt_viz`; then `mrpt_hwdrivers` | hwdrivers 13.9% -> 29.0% |
| 2026-09-09 | `mrpt_img`, `mrpt_math` (blind spots, docs, dead code) | see section 11 |

## 11. Recent API / correctness passes

### mrpt_nav: API modernization + TP-Space math (2026-09-08)

**Out-param APIs replaced by `std::optional`** (old signature kept as a
`[[deprecated]]` inline shim unless noted), matching `inverseMap_WS2TP()`:
`CParameterizedTrajectoryGenerator::getPathStepForDist()` (the old 3-arg form
also wrote the *last* path step on failure; `getPathStepForDistClamped()` now
covers that explicitly), `nav_plan_geometry_utils`'
`collision_free_dist_{segment,arc}_circ_robot()`, and
`PlannerSimple2D::computePath()`.

**Other API changes**: `ClearanceDiagram::getClearance()`'s `bool
integrate_over_path` became `enum class ClearanceQuery`;
`CPTG_Holo_Blend::PATH_TIME_STEP` (a mutable global) became the per-instance
`path_time_step` config key + `setPathTimeStep()`, and `eps` became `EPSILON`
(both no-shim breaks); `setScorePriorty()` -> `setScorePriority()`;
`updateClearancePost()` (a no-op since 2017) and
`CAbstractHolonomicReactiveMethod::Create()` (declared but never defined) were
deleted.

Eight real bugs were fixed with regression tests, the notable classes being:
two enum/bool modes swapped relative to their own docs; three unit mismatches
where normalized [0,1] TP-Space distances and raw meters were mixed at an API
boundary; an assignment placed inside the wrong `if`; and a closed form that
divided by a coordinate that is zero for any obstacle on the turn-center axis
(rewritten as a two-circle intersection).
`calc_trans_distance_t_below_Tramp_abc_numeric()` went from a 15-interval
trapezoidal rule to 16-interval Simpson (same cost, ~25x lower error) plus an
exact branch for the degenerate `b^2-4ac ~= 0` case.

New console example `mrpt_examples_cpp/nav_ptg_tpspace` walks the whole
WS -> TP-Space -> velocity-command round trip headlessly.

### mrpt_img + mrpt_math: blind spots, docs, dead code (2026-09-09)

`mrpt_img` 93.3% -> 97.9%, `mrpt_math` 95.0% -> 96.8%.

**The SIMD kernels in `mrpt_img` are live again.** `CImage.SSE2.cpp` /
`CImage.SSSE3.cpp` had been orphaned by the stb-based `CImage` rewrite: nothing
but their own unit test called them, and `scaleHalf()`/`grayscale()` documented
their `bool` return as "always false, reserved for a future SIMD fast path".
`CImage::scaleHalf()` now dispatches to `image_SSSE3_scale_half_3c8u` (3-channel
`IMG_INTERP_NN`), `image_SSE2_scale_half_1c8u` (1-channel NN) and
`image_SSE2_scale_half_smooth_1c8u` (1-channel `IMG_INTERP_LINEAR`), and
`grayscale()` to `image_SSSE3_rgb_to_gray_8u`; the `bool` return now truthfully
reports whether a fast path ran. Two constraints matter: the gray kernel
asserts both row strides are multiples of 16 bytes, which for `CImage`'s packed
rows means `width % 16 == 0` (the scale-half kernels handle the remainder
themselves), and none of them can run in place. Note this also restores MRPT
2.x's `IMG_INTERP_NN` semantics for `scaleHalf` — point sampling, where the stb
`STBIR_FILTER_BOX` path box-averages.

**Dead code removed from `mrpt_math`** (~1700 lines): the public headers
`CBinaryRelation.h`, `matrix_adaptors.h`, `MatrixBlockSparseCols.h`,
`CMonteCarlo.h` and `eigen_extensions.h`. Nothing in the repo included any of
them, and all but `eigen_extensions.h` fail to even compile standalone — they
reference `CMatrixTemplateObjects` (removed in the 3.x matrix rewrite) or
`Eigen::Matrix` without including Eigen — so no working downstream code can be
using them. Also removed: a second, unreferenced
`::intersect(TPolygonWithPlane, TPolygonWithPlane, TObject3D)` in
`geometry.cpp`, duplicating what `intersectAux()` does for the public
`intersect(TPolygon3D, TPolygon3D)`.

**Real bugs found and fixed:**

* `CImage::grayscale(ret)` documents in-place use (`ret = *this`), but
  `ret.resize()` frees the source buffer before the conversion loop reads it —
  the result was garbage plus a 2-byte heap over-read on the last pixel.
* `mrpt::math::assemblePolygons()`'s three `TObject3D` overloads collected the
  polygons already present in the input and then called the segment-based
  overload, which **overwrites** its output vector: every pre-existing polygon
  was silently dropped.
* `assemblePolygons(segments, ...)` looped `for (size_t i = 0; i < N - 1; i++)`,
  which underflows to ~2^64 iterations for an empty input — a hang, not a
  wrong answer.
* `TSegment3D::distance(TPoint3D)` returned
  `min(d(p,p1), d(p,p2), d(p, infinite line))`, so a point beyond an endpoint
  measured to the unbounded line: a point 1 m past the end of a segment
  reported distance 0. It now clamps the projection parameter to [0,1], like
  its correct 2D twin `TSegment2D::signedDistance()`. `distance(TSegment3D)`
  additionally mishandled a zero-length operand (the "almost parallel" branch
  settles on the wrong endpoint) and now delegates to the point overload.
* `MatrixVectorBase::saveToTextFile()` wrote `userHeader` with no trailing
  newline although the doc says "final end-of-line is not needed", gluing the
  header onto the first data row. With the usual `%`-prefixed header that made
  `loadFromTextFile()` skip the first row of data silently. A newline is now
  added when missing.
* `TCamera::serializeFrom()` did not reset `cameraName` for pre-v5 streams
  (unlike `nrows`/`ncols`/`distortion` in the same function), so reading a
  legacy camera into a reused object kept the previous name.

**Doc corrections**: `saveToTextFile()`'s `appendMRPTHeader` text described a
header string the code has not written for years; the SIMD kernels' doxygen
still pointed at `CImage::scaleHalfSmooth()` and `CImage::grayscaleInPlace()`,
neither of which exists in 3.x.
