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

## 9. Code Coverage Status (baseline: 2026-07-03, refreshed 2026-07-06)

A full rebuild of all 33 `modules/*` packages was done with coverage
instrumentation, followed by a full `colcon test` run (all tests passed) and a
`gcovr` line/branch report. **Goal: 90% line coverage per module.** Current
overall (2026-07-06): **51.3% lines / 38.1% branches** — well short of goal.
Per-module rows below are still the 2026-07-03 baseline except where a row is
tagged with a newer date (e.g. `mrpt_math`, `mrpt_graphs`, after their
unit-test passes).

Gotcha (2026-07-06): the system `gcov` alias (`/etc/alternatives/gcov`) may
point to an unrelated binary (observed pointing to `/usr/bin/gc`), causing
gcovr to silently mis-decode gcov output as garbage/UnicodeDecodeErrors.
Pass `--gcov-executable gcov-<major-version-matching-the-compiler>` (e.g.
`gcov-13`) explicitly to `gcovr` rather than relying on the `gcov` PATH
lookup.

To reproduce:
```bash
colcon build --base-paths modules --cmake-args -DENABLE_COVERAGE=ON -DBUILD_TESTING=ON
colcon test --base-paths modules
gcovr --root . -j$(nproc) --gcov-ignore-parse-errors=all \
  --exclude-unreachable-branches --exclude-throw-branches \
  --exclude '.*/3rdparty/.*' --exclude '.*/tests/.*' --exclude '.*_unittest\.cpp' \
  --exclude '.*/python_bindings/.*' --exclude '.*/samples/.*' \
  --json-pretty -o coverage.json build
```
(`--gcov-ignore-parse-errors=all` is needed, not just `negative_hits.warn_once_per_file`:
large repos also trip gcovr's "suspicious hits" detector, e.g. on
`mrpt_maps/src/maps/COccupancyGridMap2D_likelihood.cpp`, which otherwise aborts
the whole run with a `SuspiciousHits` exception.)

Gotcha: with symlink-install, the same header is reported twice by gcovr
(once under `modules/<pkg>/include/...`, once under the symlinked
`install/<pkg>/include/...`). Dedupe by stripping the leading `modules/` or
`install/` path segment and merging line hit-counts (max) before computing
per-file/per-module percentages, or numbers will be wrong in both directions.
`scripts/coverage_module_report.py coverage.json <module1> [<module2> ...]`
does this dedupe and prints per-file + aggregate line/branch % for the given
module(s), e.g. `scripts/coverage_module_report.py coverage.json mrpt_math`.

Gotcha (2026-07-09): re-running a coverage-instrumented test binary directly
(not via `colcon test`) without deleting old `.gcda` files first causes
`libgcov profiling error: ... overwriting an existing profile data with a
different checksum` — safe to ignore for pass/fail, but before trusting a
`gcovr` report, `find build -iname '*.gcda' -delete` and re-run `colcon test`
once to get a clean, single-run coverage capture.

Gotcha (2026-07-09): `mrpt_math` only explicitly instantiates fixed-size
matrices/vectors for a handful of dimensions (square `CMatrixFixed`: 2,3,4,6,
7,12; `CVectorFixed`: 2,3,4,5,6,7,12 — see
`mrpt_math/src/MatrixVectorBase_instantiate_{CMatrixFixed,CVectorFixed}.cpp`).
Writing a test that instantiates a template (e.g. `CKalmanFilterCapable<...>`)
with a size outside that list, such as `VEH_SIZE=1`, compiles but fails to
*link* (`undefined reference to MatrixVectorBase<...>::impl_op_...`). Pick a
supported size (2 is the smallest) for any new fixed-size-matrix-based test.

### Measuring a single module's coverage after changing it

When you only touched one module and want its updated number, it's tempting
to build+test just that module (`colcon build --packages-up-to mrpt_XXX`,
`colcon test --packages-select mrpt_XXX`) and run gcovr over just
`build/mrpt_XXX`. **This badly undercounts modules with template-heavy public
headers** (e.g. `CMatrixFixed.h`, `CMatrixDynamic.h` in `mrpt_math`): most of
those headers' instantiation coverage comes from *other* modules' tests
exercising them (mrpt_poses, mrpt_obs, mrpt_maps, ... all instantiate
matrix/point/pose templates too), not from the owning module's own tests. An
isolated single-package run only credits the owning module's own test binary,
which can read 30-40 points lower than the true, whole-repo number.

To get a number comparable to the table below, you must rebuild + retest
**all** packages (coverage instrumentation is a global CMake cache option,
so `colcon build`/`colcon test` with no `--packages-*` filter reuses existing
object files and is normally fast/incremental — a few minutes, not a full
rebuild from scratch), then run gcovr once over the whole `build` tree and
filter with `coverage_module_report.py` as above. There isn't a fast, cheap,
and accurate path — pick two.

### Coverage by module (worst first)

| Module | Covered/Total lines | Line % | Branch % |
|---|---|---|---|
| mrpt_imgui | 0/53 | 0.0% | 0.0% |
| mrpt_gui | 22/4621 | 0.5% | 0.2% |
| mrpt_hwdrivers | 913/6592 | 13.9% | 9.7% |
| mrpt_comms | 237/1013 | 23.4% | 11.2% |
| mrpt_viz | 2660/9024 | 29.5% | 17.3% |
| mrpt_kinematics | 184/482 | 38.2% | 17.9% |
| mrpt_img | 4132/10271 | 40.2%† | 30.0% |
| mrpt_graphslam | 257/611 | 42.1% | 37.3% |
| mrpt_obs | 2922/6723 | 43.5% | 26.3% |
| mrpt_topography | 172/364 | 47.3% | 27.9% |
| mrpt_maps | 5605/11648 | 48.1% | 32.7% |
| mrpt_opengl | 2035/4234 | 48.1% | 30.1% |
| mrpt_system | 1012/1900 | 53.3% | 39.6% |
| mrpt_io | 726/1292 | 56.2% | 39.9% |
| mrpt_libapps_cli | 1130/1910 | 59.2% | 38.0% |
| mrpt_libapps_gui | 952/1567 | 60.8% | 42.6% |
| mrpt_nav | 3928/6234 | 63.0% | 45.3% |
| mrpt_slam | 2778/4299 | 64.6% | 43.7% |
| mrpt_rtti | 126/176 | 71.6% | 73.5% |
| mrpt_serialization | 511/708 | 72.2% | 52.5% |
| mrpt_containers | 1639/1956 | 83.8% | 55.6% |
| mrpt_math (2026-07-05) | 6914/8070 | 85.7% | 57.5% |
| mrpt_core | 541/628 | 86.1% | 64.8% |
| mrpt_graphs (2026-07-06) | 1022/1111 | 92.0% | 76.7% |
| mrpt_poses | 6263/6787 | 92.3% | 59.8% |
| mrpt_expr | 93/100 | 93.0% | 60.2% |
| mrpt_random | 160/167 | 95.8% | 85.1% |
| mrpt_bayes (2026-07-09) | 1036/1078 | 96.1% | 77.4% |
| mrpt_config (2026-07-09) | 531/548 | 96.9% | 82.3% |
| mrpt_tfest (2026-07-07) | 633/652 | 97.1% | 73.3% |
| mrpt_typemeta | 57/57 | 100.0% | 85.1% |

† `mrpt_img` includes the vendored `src/stb/*.h` (stb_image/stb_image_resize2/
stb_image_write, public-domain third-party). Excluding those, first-party
`mrpt_img` coverage is 48.0%. Treat `src/stb/*` as out of scope for new tests.

### Weak areas, grouped by root cause

1. **Hardware drivers — `mrpt_hwdrivers` (13.9%), most of `mrpt_comms` (23.4%)**:
   inherently hard to unit-test since they talk to real serial ports/USB/GPS/
   LIDAR/cameras (`CHokuyoURG`, `CSickLaserSerial`, `COpenNI2Generic`,
   `CVelodyneScanner`, `CSerialPort`, `CNTRIPClient`, `CKinect`, etc., all at
   0%). Improving this needs a mockable transport layer (inject a fake
   `CStream`/socket) rather than plain unit tests against hardware.

2. **GUI/rendering — `mrpt_gui` (0.5%), `mrpt_imgui` (0%), and GUI-only files
   inside `mrpt_viz`/`mrpt_opengl`**: `mathplot.cpp`, `CDisplayWindow*.cpp`,
   `WxUtils.cpp`, `CWxGLCanvasBase.cpp`, `CQtGlCanvasBase.cpp`,
   `CImGuiSceneView.cpp` need a live display/OpenGL context and are 0%.
   Realistic path to improvement is extracting non-UI logic into testable
   helpers, or headless/offscreen-context tests, not brute-force unit tests.

3. **CLI apps — `mrpt_libapps_cli` (59.2% as of 2026-07-06, was 9.1%)**: some
   `rawlog-edit_*.cpp` paths remain untested. These are better suited to
   subprocess/golden-file integration tests (run the built binary against
   sample rawlogs, diff the output) than pure unit tests.

4. **Quick wins — pure-logic files at 0% with no hardware/GUI dependency**
   (highest-value gaps, ordinary unit tests would work immediately):
   `mrpt_system/src/md5.cpp`, `mrpt_graphslam/src/{CEdgeCounter,TSlidingWindow,
   CWindowObserver}.cpp`, `mrpt_obs/src/gnss_messages_novatel.cpp`,
   `mrpt_obs/src/carmen_log_tools.cpp`,
   `mrpt_viz/src/PLY_import_export.cpp`, `mrpt_viz/src/COrbitCameraController.cpp`,
   `mrpt_img/src/CImage_loadXPM.cpp`, `mrpt_slam/src/slam/
   CRejectionSamplingRangeOnlyLocalization.cpp`.

5. **Biggest single-file impact (most uncovered lines, worth prioritizing for
   raw percentage gains)**: `mrpt_viz/src/CPolyhedron.cpp` (1420 uncovered,
   pure geometry, no GUI dependency — good test target),
   `mrpt_maps/src/maps/CRandomFieldGridMap2D.cpp` (827),
   `mrpt_maps/src/maps/CPointsMap.cpp` (547),
   `mrpt_maps/src/maps/CGasConcentrationGridMap2D.cpp` (520),
   `mrpt_obs/src/CObservation3DRangeScan.cpp` (459).

6. **Near-target modules (75-90%), smallest remaining gap to close first**:
   `mrpt_containers` (`yaml.cpp` 78.8%).
   (`mrpt_graphs` and `mrpt_random` cleared this bucket as of 2026-07-06;
   `mrpt_bayes` and `mrpt_config` cleared it as of 2026-07-09, both now >96%.)

Branch coverage lags line coverage everywhere (often by 15-30 points),
indicating error-handling and edge-case branches are the norm left untested
even in files with decent line coverage — prioritize adding failure-path
tests, not just more happy-path calls.

