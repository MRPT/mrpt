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

## 10. Code Coverage Status (baseline: 2026-07-03, refreshed 2026-07-06)

A full rebuild of all 33 `modules/*` packages was done with coverage
instrumentation, followed by a full `colcon test` run (all tests passed) and a
`gcovr` line/branch report. **Goal: 90% line coverage per module.** Current
overall (2026-09-07, deduplicated the same way as
`scripts/coverage_module_report.py`): **80.9% lines**
- still short of goal, dominated by the hardware/GUI modules below.
The whole table below was re-measured on 2026-09-07; the date tags on some
rows mark when that module last had a dedicated unit-test pass, and point at
the footnote describing it.

**The GUI tests need a display**: run `colcon test` under
`xvfb-run -a --server-args="-screen 0 1280x1024x24"` (as CI does), or the
`mrpt_gui` window tests `GTEST_SKIP()` and that module reads ~1% instead of
~40%. `MRPT_SKIP_GUI_TESTS=1` forces the skip.

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
  --exclude '.*/3rdparty/.*' --exclude '.*/stb/.*' --exclude '.*/tests/.*' \
  --exclude '.*_unittest\.cpp' --exclude '.*/python_bindings/.*' --exclude '.*/samples/.*' \
  --json-pretty -o coverage.json build
```
Gotcha (2026-08-28): `--base-paths modules` deliberately excludes `apps/`, so
the `rawlog-edit` binary is never built and the ~40 `RawlogEditCLITest` cases
in `mrpt_libapps_cli` all `GTEST_SKIP()` with "rawlog-edit binary not found".
Following the recipe verbatim therefore measures that module at ~8% rather
than the ~59% recorded below. Add `apps` to the `--base-paths` list when the
`mrpt_libapps_*` numbers matter.

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

Gotcha (2026-08-02): with `gcovr` 8.6 (vs. whatever earlier version this
baseline was first taken with), the reproduce command above can now abort
mid-run with `GcovrMergeAssertionError: ... Got function
mrpt::containers::yaml_ref::operator=(double) on multiple lines: 1036, 1042`
(a header-only template function reported at two different line numbers by
different `.gcda` files, e.g. one from `mrpt_opengl`'s and one from
`mrpt_img`'s object files). Add `--merge-mode-functions=merge-use-line-min`
to the `gcovr` invocation to resolve the ambiguity instead of failing the
whole run.

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
| mrpt_hwdrivers (2026-09-07)§ | 1957/6746 | 29.0% | 20.5% |
| mrpt_gui (2026-09-06)@ | 1888/4655 | 40.6% | 30.4% |
| mrpt_opengl | 2323/4234 | 54.9% | 35.7% |
| mrpt_libapps_cli | 1129/1910 | 59.1% | 38.0% |
| mrpt_libapps_gui | 803/1288 | 62.3% | 45.8% |
| mrpt_common | 5/7 | 71.4% | 0.0% |
| mrpt_comms (2026-08-29)» | 687/906 | 75.8% | 54.4% |
| mrpt_examples_cpp | 99/128 | 77.3% | 46.3% |
| mrpt_graphslam (2026-09-06)@ | 814/997 | 81.6% | 64.4% |
| mrpt_system (2026-08-29)» | 1637/1964 | 83.4% | 59.0% |
| mrpt_rtti (2026-08-29)» | 151/176 | 85.8% | 78.1% |
| mrpt_io (2026-08-29)» | 1133/1310 | 86.5% | 70.4% |
| mrpt_viz (2026-09-07)◆ | 8684/9852 | 88.1% | 68.5% |
| mrpt_maps (2026-09-07)★ | 10631/11789 | 90.2% | 66.4% |
| mrpt_containers (2026-07-11) | 1803/1999 | 90.2% | 55.6%‡ |
| mrpt_nav (2026-08-28)¶ | 5683/6287 | 90.4% | 69.0% |
| mrpt_core | 579/638 | 90.8% | 71.9% |
| mrpt_obs (2026-09-07)★ | 6282/6857 | 91.6% | 63.1% |
| mrpt_graphs (2026-07-06) | 1030/1113 | 92.5% | 76.8% |
| mrpt_poses | 6282/6788 | 92.5% | 61.4% |
| mrpt_expr | 93/100 | 93.0% | 60.2% |
| mrpt_img (2026-07-10)† | 2856/3060 | 93.3% | 71.7% |
| mrpt_slam (2026-09-07)★ | 4065/4348 | 93.5% | 65.3% |
| mrpt_math (2026-09-07)★ | 7472/7867 | 95.0% | 64.4% |
| mrpt_bayes (2026-07-09) | 1049/1090 | 96.2% | 80.3% |
| mrpt_serialization (2026-08-29)» | 728/754 | 96.6% | 75.2% |
| mrpt_random | 162/167 | 97.0% | 88.4% |
| mrpt_tfest (2026-07-07) | 635/654 | 97.1% | 72.9% |
| mrpt_kinematics (2026-08-28)¶ | 503/518 | 97.1% | 81.2% |
| mrpt_config (2026-07-09) | 536/548 | 97.8% | 84.7% |
| mrpt_topography (2026-07-10) | 414/417 | 99.3% | 83.2% |
| mrpt_typemeta | 57/57 | 100.0% | 80.9% |

† `mrpt_img` vendors third-party `src/stb/*.h` (stb_image/stb_image_resize2/
stb_image_write, public-domain), which is out of scope for tests like any
other 3rd-party code and is excluded via `--exclude '.*/stb/.*'`, now part of
the standard reproduce command above (previously a module-scoped, one-off
filter just for this row). The 2026-07-10 pass also fixed
several bugs found via new tests: a bilinear-interpolation weight collapse and
an off-by-one edge read in `CMappedImage::getPixel`, a hue wrap-around sign
error in `rgb2hsv()`, a missing distortion-model assignment in one
`camera_geometry::undistort_points()` overload (silently a no-op), a
division-by-zero-to-NaN risk in `CImage::cross_correlation_FFT()`, an
unchecked negative-window read in `CImage::KLT_response()`, a
`saveToFile()`/doc mismatch (threw instead of returning false for unknown
extensions), a missing endpoint pixel in `CCanvas::line()`, and a missing
implementation of `CImage::saveToStreamAsJPEG()` (declared but never defined,
a link error for any caller). The `CImage.SSE2.cpp`/`CImage.SSSE3.cpp` SIMD
kernels are dead code as of the stb-based rewrite (no longer called from
`CImage.cpp`); they are covered by direct unit tests instead, which also
found and fixed a decimation remainder-loop bug and a swapped R/B luminance
weight.

‡ `mrpt_containers` line and branch % both come from
`scripts/coverage_module_report.py` (max hit-count per line, OR-merged
`(line_number, branch_index)` for branches, across duplicate
template-instantiation entries, e.g. `CDynamicGrid<double>` vs
`CDynamicGrid<int>` both mapping to the same header lines); raw un-deduped
`gcovr` CLI output for this module reads ~87% lines / ~45% branches instead.
This module-scoped run (not the whole-repo `colcon test`) is
adequate here since `mrpt_containers`' templated headers (`CDynamicGrid`,
`circular_buffer`, `ts_hash_map`, `deepcopy_ptr`) are almost entirely exercised
by this module's own tests, unlike `mrpt_math`'s matrix templates. New tests
added on 2026-07-11 also fixed two test-only misunderstandings (not source
bugs) around `yaml`'s null-node vs. null-scalar distinction: `operator[]`
throws for a null node only via the *const* overload (the non-const overload
auto-vivifies into a map), and a scalar node holding `std::monostate` (e.g.
parsed from YAML `null`) is *also* `isNullNode() == true`, which is why
`node_t::typeName()`'s scalar-visitor branch for `std::monostate` (yaml.cpp)
is unreachable dead code — `isNullNode()` intercepts first. The `shared_ptr<yaml>`
alternative in `scalar_t` remains unreachable via the public API (no
construction call sites), same as noted before; left in place as it is part of
the variant's public type and removing it would be an ABI-affecting change.

§ `mrpt_maps` was raised from 48.1%/32.7% (2026-07-03 baseline) to 83.0%/59.6%
on 2026-07-17, in two passes. Pass 1 (74.5%/53.9%) added new/extended tests
for `CRandomFieldGridMap2D` (35%→86%, all 5 map-type representations),
`CPointsMap` (51%→80%), `CGasConcentrationGridMap2D` (13%→84%, incl. the
wind-advection simulation path), `CBeaconMap`/`CBeacon` (76%→93% / new),
`COccupancyGridMap2D_insert.cpp` (30%→92%), the Voronoi/critical-points code,
`CObservationPointCloud` (new), and `customizable_obs_viz.cpp` (0%→97%).
Pass 2 (83.0%/59.6%) covered the remaining gaps: `COctoMap`/`CColouredOctoMap`
(21%/15%→76%/95%, incl. the shared `COctoMapBase` template — insertion for
2D/3D scans, `castRay`, `getPointOccupancy`, colour update via
`SET`/`AVERAGE`/`INTEGRATE`), `CVoxelMap`/`CVoxelMapRGB`/`CVoxelMapOccupancyBase`
(27%/22%/33%→85%/77%/94%, incl. `remove_voxels_farther_than` and the
range-image-based colour-scan path), `COccupancyGridMap3D` (51%→82%, incl.
the `TInsertionOptions`/`TRenderingOptions` structs and the `determineMatching2D`/
`compute3DMatchingRatio`/`internal_computeObservationLikelihood` stubs — see
below), `CMultiMetricMap` (75%→90%, `determineMatching2D`, `getAsSimplePointsMap`,
`mapByIndex`, serialization), and a new `CPlanarLaserScan` test file (0%→96%).
Real bugs found and fixed along the way: (1) `CGasConcentrationGridMap2D::
getWindAs3DObject()` and `::simulateAdvection()` both dereferenced
`CDynamicGrid::cellByPos/cellByIndex()`'s result unchecked, segfaulting
whenever a wind-grid query point/index fell outside the grid (a boundary
point at exactly the grid edge, or index drift when the main grid resizes
but the wind sub-grids don't); (2) `simulateAdvection()`'s variance-update
loop treated the *compressed* `m_stackedCov` (N rows × small W-window
columns) as if it were a full N×N matrix, writing out-of-bounds columns and
corrupting the heap (`double free or corruption`) on any grid larger than
the window; (3) `TInsertionOptions::loadFromConfigFile()` marked
`gasSensorLabel`/`gasSensorType`/`windSensorLabel`/`useWindInformation`/
`advectionFreq` all `failIfNotFound=true` despite each having a documented
default (making the defaults dead code and any config file missing one of
these keys throw), and separately passed the string literal `"false"` where
`read_bool()` expects an actual `bool` default (a non-null pointer coerces
to `true`, silently inverting the intended default); (4)
`COccupancyGridMap2D::findCriticalPoints()` computed `temp_x.size() - 1`
with unsigned `size_t` arithmetic, underflowing to `SIZE_MAX` and reading
wildly out of bounds whenever zero or one critical-point candidates were
found (the common case for any corridor/room without branch points — i.e.
most maps); (5) `COccupancyGridMap2D::getVoronoiClearance()` dereferenced
`cellByIndex()`'s result unchecked, segfaulting for any negative or
out-of-range `(cx,cy)`, reachable from `findCriticalPoints()`'s
`x-2`/`y-2` neighbor scan. `determineMatching2D()`'s bounding-box overlap
pre-filter (not expanded by `maxDistForCorrespondence`/
`maxAngularDistForCorrespondence`) can also incorrectly early-reject valid
correspondences when the two maps' raw bounding boxes don't quite touch;
identified but *not* fixed (a hot, SIMD-optimized path used throughout
ICP/SLAM, too risky to patch without dedicated validation) — the affected
test was reshaped to avoid the case instead. Separately, MRPT's PLY point
cloud writer (`mrpt::viz::PLY_import_export.cpp`) only round-trips a single
grayscale "intensity" property (`(R+G+B)/3`, broadcast back to all three
channels on import), not full per-channel RGB — not a bug, just a
narrower-than-expected feature scope that a new test initially assumed
incorrectly. `COccupancyGridMap3D::determineMatching2D()`,
`::compute3DMatchingRatio()`, and `::internal_computeObservationLikelihood()`
are all unimplemented stubs (`THROW_EXCEPTION("Implement me!")`, pre-existing,
self-documented) — new tests assert the current (throwing) behavior rather
than a working implementation; actually implementing them is a real
algorithmic task, not a bug fix, and was left for a dedicated session.
`CVoxelMapRGB`'s 3D-scan colour path and `CColouredOctoMap`'s 3D-scan path
both unproject via a range image + camera intrinsics (`hasRangeImage`), NOT
via `hasPoints3D`/`points3D_x/y/z` like most other 3D-scan consumers in this
module — a manually-constructed `CObservation3DRangeScan` for tests needs
`cameraParams.setIntrinsicParamsFromValues(...)` plus a filled `rangeImage`
matrix, or `unprojectInto()` silently yields zero points. A parallel 8-agent
test-writing fleet was used for the first half of this session; all 8 hit
the account's session usage limit mid-task, but each agent's already-applied
file edits survived intact (edits are atomic) — after registering the
surviving new files and fixing the crashes/config bugs above, a second,
solo pass covered the remaining gaps the fleet never reached
(`CColouredOctoMap`, `COctoMap`, `CVoxelMap`/`CVoxelMapRGB`/
`CVoxelMapOccupancyBase`, `COccupancyGridMap3D`, `CMultiMetricMap` extra
coverage, and a new `CPlanarLaserScan` test file).

`mrpt_maps` was further raised from 83.0%/59.6% to 86.6%/62.5% on 2026-08-03
(pass 3), targeting the lowest-coverage files remaining after pass 2:
`COccupancyGridMap2D_getAs.cpp` (49%→96%, all `TGetAsImageParams` combinations,
`getAsImageFiltered()`'s gaussian/median filter branches, `getVisualizationInto()`
enabled/disabled, `getAsPointCloud()` border-cell selection), `_io.cpp`
(55%→93%, bitmap save/load round-trip, `loadFromBitmap()`'s centered-origin
sentinel, `saveAsBitmapTwoMapsWithCorrespondences()`,
`saveMetricMapRepresentationToFile()`, and the ROS map-server YAML `scale`/
`negate`/missing-image/invalid-`mode` branches, using new fixture files
`yaml_32_{scale,negate,badmode,missing_image}.yaml`), `_simulate.cpp`
(53%→100%, out-of-range/short-max-range rays, noisy rays,
`laserScanSimulator()` decimation, `laserScanSimulatorWithUncertainty()` for
both `sumUnscented`/`sumMonteCarlo` and its unknown-method throw path),
`_likelihood.cpp` (80%→90%, a parameterized test now drives all 7
`TLikelihoodMethod` values through the real public dispatch instead of only
`lmLikelihoodField_Thrun`/`_II`), `COctoMap.cpp` (75%→94%, all 7
`COctoMapVoxels::visualization_mode_t` coloring branches plus
`generateGridLines` and the const `getMetricSize/Min/Max` overloads),
`CVoxelMapRGB.cpp` (76%→91%, serialization round-trip,
`remove_voxels_farther_than`, the `ray_trace_free_space` toggle, an empty-3D-scan
early return), `CRandomFieldGridMap3D.cpp` (79%→92%, `TInsertionOptions`
load/dump, `saveAsCSV()` with both mean+stddev files and its failure path,
serialization round-trip, `insertIndividualReading(..., update_map=true)`),
and `CPointsMap_crtp_common.h` (68%→97%, the `CObservation3DRangeScan`
overload of `loadFromRangeScan()` — `insertInvalidPoints` and the
no-points-3D early return — which no test in the module exercised at all
before this pass). A parallel 5-agent test-writing fleet was used for the
first half of this pass; all 5 hit the account's session usage limit
mid-task (same failure mode as the 8-agent fleet in pass 2), but each
agent's already-applied file edits survived intact — a solo pass then fixed
the resulting test failures and covered the remaining gaps (`_io.cpp`,
`COctoMap.cpp`'s coloring-mode variants, `CRandomFieldGridMap3D.cpp`,
`CPointsMap_crtp_common.h`'s 3D-scan overload, and a small voronoi
three-way-junction/isolated-obstacle addition). Two new `_unittest.cpp`
files (`COccupancyGridMap2D_getAs_unittest.cpp`, `_io_unittest.cpp`) had to
be added by hand to `mrpt_maps/CMakeLists.txt`'s explicit
`LIB_UNIT_TEST_SOURCES` list — unlike some other modules, this list is not
glob-based, so a new `*_unittest.cpp` file silently never runs (and never
moves the coverage numbers) until it's registered there. Real bugs found
and fixed along the way: (1) `CDynamicGrid3D::dyngridcommon_readFromStream()`
(`mrpt_containers`, used by `CRandomFieldGridMap3D` and `CLogOddsGridMap3D`)
updated `m_size_x`/`m_size_y`/`m_size_z` from the stream but never recomputed
the cached `m_size_x_times_y` z-axis stride, so deserializing into an object
whose grid dimensions differ from what it had before (e.g. any freshly
default-constructed object) left cell-index lookups using a stale stride,
silently reading out-of-bounds heap memory instead of the intended cell — a
memory-safety bug, not just wrong data; (2)
`COccupancyGridMap2D::computeObservationLikelihood_ConsensusOWA()` had its
"is this a `CObservation2DRangeScan`?" branch condition inverted
(`if (IS_CLASS(...))` instead of `if (!IS_CLASS(...))`), so the method
returned a constant `1e-3` fallback for its only supported observation type
and fell through to an unconditional `dynamic_cast`-triggered `bad_cast` for
every other type — `lmConsensusOWA` was completely non-functional before this
fix; (3) `TLaserSimulUncertaintyParams::method`'s doc comment claimed the
default was `sumMonteCarlo` while the member initializer actually defaults to
`sumUnscented` (doc-only fix). `computeObservationLikelihood_ConsensusOWA()`
also has two more `ASSERT_`-guarded preconditions that are easy to trip by
accident rather than fix (a hot likelihood-evaluation path, same "too risky
to patch without dedicated validation" reasoning as `determineMatching2D`
above): `OWA_weights.size()` must not exceed the scan's point count (default
is 100 weights), and every scan point, when re-projected from whatever pose
is being evaluated, must land within the grid bounds or a `nCells > 0` assert
fires — the affected tests use a generously oversized grid and a smaller
`OWA_weights` vector to stay clear of both, rather than patching the source.

`mrpt_viz` was raised from 29.5%/17.3% (2026-07-03 baseline) to 68.4%/50.1% on
2026-08-02. `mrpt_viz` has zero OpenGL dependency (it's the abstract scene-graph
description consumed by `mrpt_opengl`, not the other way round), so almost all
of the gain came from new, direct, non-rendering unit tests added under
`modules/mrpt_viz/tests/`: `CPolyhedron_unittest.cpp` (the session's single
biggest target, `CPolyhedron.cpp` going from a large uncovered file to
91.6%/82.8%, exercising every `Create*` platonic/Archimedean/Catalan/Johnson
solid factory plus `getDual`/`truncate`/`cantellate`/`augment`/`rotate`/`scale`),
`PLY_import_export_unittest.cpp`, `COrbitCameraController_unittest.cpp`,
`Scene_Viewport_unittest.cpp`, `CVisualObject_unittest.cpp` (base class +
the four `VisualObjectParams_*` mixins), `CGeneralizedEllipsoid_unittest.cpp`
(the `CEllipsoid2D/3D`, `InverseDepth2D/3D`, `RangeBearing2D` family),
`pose_pdfs_unittest.cpp`, `StockObjects_unittest.cpp`, and
`MiscVisualObjects_unittest.cpp` (`CAxis`, `CVectorField2D/3D`, `CFrustum`,
`CSetOfObjects`, `CCamera`, `TTriangle`, `COctoMapVoxels`, `CSkyBox`, `CDisk`,
`CBox`, `CGridPlaneXY/XZ`, `CTexturedPlane`). A smaller share came from
extending the `mrpt_opengl` reference-image suite with
`CFBORender_ExtraEllipsoidsAndMesh_unittest.cpp` (the inverse-depth/
range-bearing ellipsoid parameterizations, `CGridPlaneXZ`, `CMesh3D`), the
same offscreen-render/PNG-diff technique as the pre-existing `CFBORender_*`
tests — that file has to live under `mrpt_opengl/tests/` rather than
`mrpt_viz/tests/` given the one-way dependency, but the coverage it generates
is still credited to `mrpt_viz`'s `.gcda` files since coverage instrumentation
is per-compiled-object, not per-test-binary. Real bugs found and fixed along
the way: (1) `CPolyhedron::InitFromVertAndFaces()` never assigned
`m_Vertices`/`m_Faces` from its parameters, only from whatever the member
variables already held — harmless for the two constructors that pre-populate
those members via their initializer list before calling it, but the
`CPolyhedron(vertices, vector<vector<uint32_t>> faces)` convenience
constructor left them empty, silently producing a 0-vertex/0-face polyhedron;
(2) `PLY_Importer::loadFromPlyFile()` segfaulted (null-pointer dereference in
`ply_close()`) instead of returning `false` for a nonexistent/unreadable
file, since `ply_open_for_reading()` returning `nullptr` on `fopen()` failure
was never checked; (3) `CMesh3D::loadMesh()`'s (raw-pointer overload)
triangle face-normal computation read vertex index slot `[3]`, which for a
triangle (as opposed to a quad) is the unused `-1` sentinel cast to
`uint32_t` (`4294967295`), producing an out-of-bounds `std::vector` read and
a segfault for any mesh with `enableFaceNormals(true)` and at least one
triangle face — a `CMesh3D::loadMesh(is_quad, face_verts, vert_coords)`
matrix-based overload right below it had the equivalent code correct, which
is what gave away the right fix. Also fixed: the `CreateJohnsonSolidWithConstantBase()`
API-doc example table in `CPolyhedron.h` listed `"C+PRC-"` for an 8-vertex
rhombicuboctahedron, but the parser requires the downward-cupola component
(`C-`) first and the upward one (`C+`) last (confirmed against
`CreateRandomPolyhedron()`'s own internal usage) — the doc string was
backwards and has been corrected to `"C-PRC+"`. Remaining gaps below ~55%:
`CAnimatedAssimpModel.cpp`/`CAssimpModel.cpp` (0%, need an external 3D model
file plus system assimp — not attempted), `CTextMessageCapable.cpp` (0%,
on-screen text-overlay list, unexercised by any offscreen-render or unit
test), `opengl_fonts.h` (0%, embedded glyph bitmap data), and
`CSetOfTexturedTriangles.cpp` (2.1%, needs a rendering-based test since its
logic is mostly buffer-upload bookkeping).

¶ `mrpt_nav` was raised from 63.0%/45.3% (2026-07-03 baseline) to
90.0%/69.3% on 2026-08-28, together with `mrpt_kinematics`
(38.2%/17.9% → 96.8%/83.5%), which had **no C++ unit tests at all** —
only a python-bindings smoke test, so its `CMakeLists.txt` had no
`LIB_UNIT_TEST_SOURCES` list at all and one had to be added. New test files:
`mrpt_kinematics/tests/{CVehicleVelCmd,CVehicleSimul,CKinematicChain}_unittest.cpp`
and `mrpt_nav/tests/{TWaypoint,PTG_variants,CAbstractNavigator,nav_misc,
planners_and_logs,holonomic_config,rnav_variants,nav_interfaces}_unittest.cpp`.
`CPTG_DiffDrive_CC`/`_CS`/`_CCS` had never been instantiated by any test
(~2% each) and `impl_renderMoveTree.h` was at 0%.

Real bugs found and fixed along the way: (1) `CVehicleVelCmd`'s copy
constructor delegated to `operator=()`, which dispatches pure virtual methods
while the derived object is still under construction — copy-constructing *any*
velocity command aborted the process; (2)
`CAbstractNavigator::internal_onStartNewNavigation()` cleared the cached pose
history but left `m_last_curPoseVelUpdate_robot_time` untouched, so the
`updateCurrentPoseAndSpeeds()` call right after it was skipped by its 20 ms
minimum-period throttle and the following `ASSERT_(!m_latestPoses.empty())`
threw on the first navigation step of *every* waypoint mission and *every*
relative-target navigation; (3) `performNavigationStepNavigating()` ended with
`m_navigationState = prevState;`, undoing the transitions it had just decided,
so neither an exception nor `doEmergencyStop()` could ever leave the navigator
in `NAV_ERROR` (the assignment was meant for `m_lastNavigationState`, and it
was also what masked bug (2)); (4)
`CWaypointsNavigator::checkHasReachedTarget()` dereferenced a possibly-null
waypoint pointer via `(wp == nullptr && ...) || (wp->reached)`; (5)
`CParameterizedTrajectoryGenerator::Alpha2index()` discarded `wrapToPi()`'s
return value, clamping out-of-range directions to the first/last path instead
of wrapping; (6) `CMultiObjectiveMotionOptimizerBase::decide()` returned `-1`
from a `std::optional<size_t>` function on a formula compile error — an
*engaged* optional holding `SIZE_MAX`, indexed out of bounds by the caller;
(7) its `clear()` kept the variable table, so the next `decide()` threw
"Expression name already exists as an input variable"; (8) `CLogFileRecord`'s
legacy (pre-v15) deserialization wrote velocity components into the wrong
slots; (9) `CReactiveNavigationSystem3D::saveConfigFile()` never called the
`CAbstractPTGBasedReactive` implementation, so a saved 3D config was an
unloadable stub; (10) `PlannerSimple2D::computePath()`'s endpoint bounds check
read `!(originInside || !targetInside)`, which only flagged
origin-outside-*and*-target-inside and let an out-of-grid target fall through
into the search.

Gotchas for future passes on these modules:

* `mrpt_nav`'s `LIB_UNIT_TEST_SOURCES` is an explicit list (like `mrpt_maps`',
  unlike glob-based modules): a new `*_unittest.cpp` silently never runs until
  it is registered there.
* `rnav_unittest.cpp`'s helper `return`s silently when the shared
  `navigation-ptgs/*.ini` files are missing *and* swallows every exception, so
  it can pass while testing nothing. New reactive-navigation tests build their
  configuration with `mrpt::config::CConfigFileMemory` instead.
* Reactive navigation tests must advance the robot's **navigation time**
  (`CRobot2NavInterface::getNavigationTime()`), not only the wall/simulated
  clock: `updateCurrentPoseAndSpeeds()` throttles to one query per 20 ms of
  robot time, and a mock returning a constant leaves the pose cache empty.
* `CHolonomicFullEval::navigate()` asserts `ni.clearance != nullptr`;
  `CHolonomicVFF`/`CHolonomicND` ignore that field.
* `CPTG_DiffDrive_*` need a polygonal robot shape in the config
  (`shape_x0`/`shape_y0`/...) or `initialize()` throws "Robot shape was not
  defined"; `CPTG_Holo_Blend` takes `robot_radius` instead and rejects a
  polygonal one. `setRefDistance()` throws on a collision-grid PTG once
  initialized.
* With a `CPTG_Holo_Blend`, a waypoint left at `TWaypoint`'s default
  `speed_ratio = 1.0` yields no viable movement at all and the mission times
  out; `rnav_variants_unittest.cpp` sets an explicit `0.05`. Not investigated
  further.
* `CWaypointsNavigator::m_last_alignment_cmd` is set in the constructor and
  never updated when an alignment command is issued, so the "give the
  alignment some time to finish" wait actually measures time since the
  navigator was constructed. Left as-is: fixing it adds a real dwell at every
  waypoint with a heading.
* `CVehicleSimulVirtualBase::setCurrentOdometricPose()` is templated but does
  not accept a `mrpt::poses::CPose2D` (there is no `TPose2D` constructor from
  it); pass `.asTPose()`.

¤ Second coverage pass of 2026-08-28, on the four largest remaining pure-logic
gaps after `mrpt_nav`/`mrpt_kinematics` (see ¶). Whole files that had never
had a single assertion run against them: `mrpt_graphslam/src/TSlidingWindow.cpp`
(0%), `.../CEdgeCounter.cpp` (0%), `mrpt_system/src/md5.cpp` (0%),
`mrpt_slam/src/slam/CRejectionSamplingRangeOnlyLocalization.cpp` (0%), and
`mrpt_io/src/vector_loadsave.cpp` (16%).

Real bugs found and fixed: (1) `TSlidingWindow::getMean()` divided by zero on
an empty window, returning NaN -- and since every comparison against a NaN is
false, `evaluateMeasurementAbove()` was then stuck at `false` for *any* input;
(2) `TSlidingWindow::getStdDev()` normalized by the window *capacity* instead
of the number of measurements held, so a partially-filled window
systematically under-reported sigma (and disagreed with `getMean()`, which
uses the sample count); (3) `TSlidingWindow::resizeWindow()` invalidated the
mean/median caches but never the std-dev one -- stale after a shrink, and on
a grow it invalidated nothing at all even though the reported value depended
on `m_win_size`; (4) `CEdgeCounter::clearAllEdges()` reset every counter
except `m_unique_edges`, so a "cleared" instance still reported a stale
unique-edge total; (5) `mrpt::system::md5(const std::vector<uint8_t>&)` used
`&str[0]`, an out-of-bounds access for an empty vector whose resulting
pointer then tripped the `ASSERT_(data)` in the overload it delegates to --
so `md5(empty_vector)` threw while `md5(empty_string)` returned the correct
digest; (6,7,8) `mrpt::io::vectorNumericFromTextFile()` discarded `fscanf`'s
return value in its default (`byRows == false`) path -- `(!byRows) ||`
short-circuits -- so a failed read still pushed the stale `number` and an
empty file yielded `{0.0}`; it also never cleared its output vector (unlike
`loadTextFile()`/`loadBinaryFile()`, which both do) and leaked the `FILE*`
that every sibling function in the same file closes.

Notable non-result: `CRejectionSamplingRangeOnlyLocalization.cpp` was the
largest 0% file in the repo (117 lines), but the 10 new tests covering the
beacon-height projection, sensor-offset compensation and two-circle
intersection geometry all passed first try. It was simply untested, not
broken.

Left documented rather than changed, as the intent is genuinely ambiguous:
`CEdgeCounter::addEdge(name, is_loop_closure=true, is_new=true)` throws when
the edge type already exists but silently drops the loop-closure count when
it is new -- same arguments, opposite behavior depending on prior state.

`CControlledRateTimer`'s header docs were corrected to match the
implementation rather than the reverse: the low-pass filter is
`estimation = a0*former_estimation + (1-a0)*input` (the doc had the weights
swapped), and the documented defaults for `Ti` (0.0194) and `a0` (0.9)
disagreed with the member initializers (0.1 and 0.99). The control law itself
was left untouched -- it is the sensible reading for a *low*-pass filter and
the PI gains were presumably tuned against it. A new
`CControlledRateTimer_unittest.cpp` pins both the documented defaults and the
filter's weighting (with `a0 == 1` the estimate must ignore the measurement
entirely and stay on the set-point), so the two cannot drift apart again.

» Coverage pass of 2026-08-29 on the five remaining pure-logic modules below
90%: `mrpt_serialization` (72.2% -> 96.6%), `mrpt_rtti` (71.6% -> 85.8%),
`mrpt_comms` (23.4% -> 75.8%), `mrpt_io` (63.6% -> 86.5%) and `mrpt_system`
(61.7% -> 82.6%). `mrpt_serialization` had **no unit tests of its own at
all** before this (its `CMakeLists.txt` said so); it now has them, using only
`std::stringstream`/`std::vector<uint8_t>` archives.

Two testing techniques worth reusing:

* `mrpt_comms/tests/comms_test_server.{h,cpp}` runs a one-shot local TCP
  server built on `CServerTCPSocket`, which lets `net_utils`' HTTP client be
  driven end-to-end (200/404, the NTRIP `SOURCETABLE` answer, chunked
  transfer encoding, basic auth, POST bodies) with no network access:
  `net_utils.cpp` went from 2.1% to 90.3%.
* `mrpt_comms/tests/CSerialPort_unittest.cpp` opens a pseudo-terminal
  (`posix_openpt`) and treats it as a serial device, so `open`/`setConfig`/
  `Read`/`Write`/`ReadString` all run against real termios without hardware:
  `CSerialPort.cpp` went from 0% to 68.2%. Note that `CSerialPort::Write()`
  ends in `tcdrain()`, which on macOS blocks until the far end consumes the
  data, so any such test must read from the master concurrently.

Real bugs found and fixed: (1) `CArchive::sendMessage()` wrote the payload
length low byte first while `receiveMessage()` and the documented frame
format both expect the high byte first, so no message of 256 bytes or more
could ever be received back; it also accepted payloads larger than the 16-bit
length field, overflowing its fixed frame buffer past ~64 KiB; (2)
`receiveMessage()` treated a complete frame with an empty payload as an
end-of-stream, rejecting it; (3) `operator<<(const std::monostate&)` writes
no version byte but the header parser only skipped that byte for `"nullptr"`,
so an empty `std::variant` could not be deserialized; (4)
`zip::decompress()`'s `std::vector` overload passed an *uninitialized* length
to zlib's `uncompress()`, which takes it as the capacity of the output
buffer; (5) `mrpt::system::getFileSize()` is documented to return `size_t(-1)`
on error but used the throwing overload of `std::filesystem::file_size`, so
`CFileGZInputStream::open()` and `zip::decompress_gz_file()` -- both
documented to return `false` -- threw instead; (6) `CMemoryStream::Seek()`
used the `Origin` enumerator in place of `Offset` when seeking from the end,
and clamped to the *allocated* size minus one, which underflows on an empty
stream; (7) `CStream::getline()` left a stray unwritten byte in the output on
EOF; (8) `CClientTCPSocket::connect()` created the socket and then threw from
every later failure path without closing it, leaving `isConnected()` lying;
(9) `http_request()` never trimmed its read buffer back to what was actually
received, so a 13-byte body came back as a ~1.4 kB vector; (10)
`consoleColorAndStyle()` had its stream selection inverted, so
`COutputLogger`'s error-level color codes went to stdout while the text went
to stderr; (11) `CFileSystemWatcher`'s destructor never joined the watch
thread it starts on Windows, so destroying one called `std::terminate()` --
the new tests are the first code to ever destroy one. `mrpt::io::CompressionType`
was also defined twice, identically, in `detect_compression.h` and
`compression_options.h`, so including both headers did not compile.

Left documented rather than changed:

* `CClientTCPSocket::connect()` implements the "wait until the connection
  attempt completes" step only for Linux (`epoll_wait`) and Apple
  (`select`) -- there is no Windows branch, so a connection to a closed port
  is reported as established there and the first `writeAsync()` then blocks
  forever (its `select()` only watches the write set, while Winsock signals a
  failed connect through the exception set). The two tests that depend on a
  refused connection being noticed `GTEST_SKIP()` on Windows.
* `CFileGZInputStream(fileName)` and `CCompressedInputStream(fileName)` both
  document `\exception std::exception If there's an error opening the file`
  but ignore `open()`'s result, silently yielding a closed stream; their
  output counterparts do throw. Making them throw would change the contract
  of `zip::decompress_gz_file()`, which relies on the current behavior.
* `mrpt_rtti`'s remaining uncovered lines are almost entirely the deferred
  class-registration queue (`pending_class_registers()`,
  `queue_register_functions_t`, and the drain loop in
  `registerAllPendingClasses()`): nothing in the repo ever pushes to it, so
  it is dead code left over from MRPT 1.x's `CLASS_INIT` mechanism.
  Removing it would take the module to ~94%.

× Coverage pass of 2026-08-31 on `mrpt_slam` (67.9% -> 90.2%), the last
pure-logic module still far from the goal. New tests avoid dataset files by
simulating everything: `mrpt_slam/tests/slam_synthetic_room.h` builds a closed
10x10 m gridmap and simulates 2D scans and odometry actions from it (shared by
the ICP-builder, RBPF and MCL tests), while the EKF-SLAM tests drive
`CLandmarksMap::simulateRangeBearingReadings()` over a handful of landmarks.
Files that had never had a single assertion run against them:
`src/slam/CLandmarksMap.cpp` (0%) and `src/slam/observations_overlap.cpp` (0%).

Real bugs found and fixed: (1) `observationsOverlap()`'s `CSensoryFrame`
overload declared its relative-pose argument `[[maybe_unused]]` and never
forwarded it, so `CIncrementalMapPartitioner`'s `smOBSERVATION_OVERLAP`
similarity compared every keyframe pair as if co-located; (2)
`CIncrementalMapPartitioner::addMapFrame()` passed the *same* relative pose to
both directions of its symmetrized similarity, but the callback always expects
"kf2 with respect to kf1", so the swapped evaluation needs the inverse -- this
moves one keyframe between partitions in the malaga dataset test, whose
expected output was updated; (3) `removeSetOfNodes(..., changeCoordsRef=true)`
composed `+p` instead of `-p`, doubling the first node's coordinates instead of
moving it to the origin as documented (it now reuses
`changeCoordinatesOriginPoseIndex(0)`); (4) two `TOptions` entries were loaded
with a *quoted* first argument to `MRPT_LOAD_HERE_CONFIG_VAR`, which
stringifies it again, so `minDistForCorrespondence`/`minMahaDistForCorrespondence`
could never be read from a config file; (5) `mrpt::maps::CLandmarksMap` is
`IMPLEMENTS_SERIALIZABLE` but was never registered in
`registerAllClasses_mrpt_slam()`, so it could not be deserialized;
(6) `TSetOfMetricMapInitializers::saveToConfigFile()` (in `mrpt_obs`) wrote a
format `loadFromConfigFile()` cannot read -- no `<class>_count` keys and
section names without the map index or the `_creationOpts` suffix -- so the
round trip always yielded zero maps; (7) `CMultiMetricMapPDF::getLastPose()`
never set its `is_valid_pose` output on the success path, unlike the two
sibling implementations; (8) `CMetricMapBuilderICP::saveCurrentEstimationToImage()`
dereferenced the gridmap pointer right after null-checking it, segfaulting when
the multi-metric map has no gridmap; (9) the range-only (beacon) branch of
`CMultiMetricMapPDF::prediction_and_update_pfOptimalProposal()` only ever set
`firstEstimateRobotHeading` in the no-odometry path and in the unreachable SOG
sub-method, so with odometry present it always hit the assert guarding it --
RO-SLAM with the exact optimal proposal could not run at all. It also printed
the drawn position to `std::cout` on every particle; that is now a debug-level
log message.

Both `CRangeBearingKFSLAM::TOptions::dumpToTextStream()` and its 2D
counterpart silently omitted every noise parameter they load
(`std_sensor_*`, `stds_Q_no_odo`, `std_odo_z_additional`, ...); they now print
them.

Worth knowing for future tests here:

* `CMetricMapBuilderRBPF::TConstructionOptions::loadFromConfigFile()` reads
  `insertionAngDistance_deg`, but `CMetricMapBuilderICP::TConfigParams` reads
  `insertionAngDistance` (already in degrees). The KF-SLAM classes'
  `loadOptions()` always read from the section named `RangeBearingKFSLAM`,
  whatever the file is called.
* `CRangeBearingKFSLAM2D` asserts `pitch == 0` on every observation, so a
  simulated `CObservationBearingRange` for it must use a zero pitch *noise*,
  not just coplanar landmarks.
* `CMonteCarloLocalization2D`/`3D` do not implement `pfOptimalProposal` (only
  the standard and the two auxiliary variants); `CParticleFilter` throws for
  it. An adaptive (KLD) sample size additionally requires
  `resamplingMethod = prMultinomial`.
* With known landmark IDs, the EKF-SLAM filters skip data association
  altogether: `getLastDataAssociation().predictions_IDs` stays empty and only
  `results.associations` is filled from the IDs.
* The beacon branch of the RBPF optimal proposal only handles `pdfGauss`/
  `pdfSOG` beacons, so a `CBeaconMap` feeding it needs
  `insertionOpts.insertAsMonteCarlo = false`.
* Do **not** timestamp simulated observations/actions with one
  `mrpt::Clock::now()` call per step: on Windows its resolution (~15 ms) is
  coarse enough that consecutive calls return the *same* value, and
  `CRobot2DPoseEstimator::processUpdateNewOdometry()` then drops every update
  ("Diff. in timestamps between odometry should be >0"), so the ICP builder's
  pose never advances. `slam_synthetic_room.h::nextTimestamp()` hands out
  timestamps 100 ms apart instead; give the action and the observation of the
  same step the *same* one, or the pose gets extrapolated twice.
* `RecursiveSpectralPartition()` returns a stable set of clusters, but their
  order depends on the sign of the eigenvector and differs between platforms:
  sort them before comparing.

@ Coverage pass of 2026-09-06 on `mrpt_graphslam` (73.3% -> 81.6%) and
`mrpt_gui` (0.5% -> 40.6%), the last two modules with whole files that had
never had a single assertion run against them.

`mrpt_gui` had **no `tests/` directory at all**. It now has one, and the
window classes are tested headlessly: `xvfb-run` supplies a virtual X display
and Mesa's software rasterizer, which is enough to open and drive
`CDisplayWindow`, `CDisplayWindow3D` and `CDisplayWindowPlots`. That single
change carried `mathplot.cpp` 0% -> 33%, `WxSubsystem.cpp` 4.7% -> 67%,
`CDisplayWindow3D.cpp` 0% -> 74%, `CDisplayWindow.cpp` 0% -> 71% and
`CDisplayWindowPlots.cpp` 0% -> 65%. `.github/workflows/build-linux.yml`
installs `xvfb`/`libgl1-mesa-dri` and wraps `colcon test` in `xvfb-run`;
`modules/mrpt_gui/package.xml` gained a matching `<test_depend>xvfb</test_depend>`
(note that `test_depend` alone changes nothing - only rosdep reads it, and the
ROS buildfarm skips this repo's tests entirely, see section 9 - so the CI
wrapper is what actually makes them run).

Worth knowing before adding more GUI tests:

* Do **not** pixel-compare window screenshots. On a virtual display there is
  no compositor, the window is never mapped, and reading its pixels back
  yields a uniformly black image even though rendering did happen (the FPS
  counter advances normally). Assert that a frame was grabbed and that it has
  a plausible size instead. Reference-image comparisons belong in
  `mrpt_opengl`'s offscreen EGL/FBO tests, which need no display at all.
* The grabbed frame is the *client* area, which under a real window manager
  is smaller than the requested outer size (title bar), while under Xvfb it
  matches exactly - so exact dimensions cannot be asserted either.
* `tests/gui_test_common.h` provides `SKIP_IF_NO_GUI()`; every window test
  must use it so the suite still passes with no display, and
  `MRPT_SKIP_GUI_TESTS=1` forces that path. Note the macOS and Windows CI
  jobs build with `-DDISABLE_WXWIDGETS=ON`, so the window tests skip there
  too - only the Linux jobs actually exercise them.
* `mrpt/gui/WxUtils.h` pulls in wxWidgets headers, but the library links
  wxWidgets *privately*, so the test target needs an explicit
  `target_link_libraries(test_mrpt_gui PRIVATE imp_wxwidgets)`.
* `CImage::at<T>()` is a raw `reinterpret_cast`: `at<mrpt::img::TColor>()` on
  a 3-channel image reads/writes 4 bytes over a 3-byte pixel and corrupts the
  heap on the last pixel. Use `at<uint8_t>(x, y, channel)`.

Real bugs found and fixed: (1) `mrpt::system::CTicTac` declared its timestamp
storage `alignas(16)` for no reason (it is only ever reinterpreted as
`struct timespec`/`LARGE_INTEGER`, both 8-byte aligned). That alignment
propagated through `CTimeLogger` up to
`mrpt::graphslam::CRegistrationDeciderOrOptimizer`, which is used as a
*virtual base*; GCC then compiled its member functions assuming `this` is
16-byte aligned and emitted `movdqa`, while the virtual-base subobject inside
a derived class sits at an offset that is only 8-byte aligned - so simply
constructing a `CFixedIntervalsNRD` **segfaulted in any optimized build**,
i.e. the whole `graphslam-engine` app was broken in Release. Same failure mode
as the one already documented in `mrpt::math::CMatrixFixed`; a `static_assert`
now pins the alignment. (2)
`CNodeRegistrationDecider::registerNewNodeAtEnd()` seeded the *root* node with
`getCurrentRobotPosEstimation()` instead of the origin, so the motion
accumulated before the first registration was applied twice: node 1 ended up
at 2x the travelled distance and the whole graph was offset. (3) The three
decider `TParams` structs (`CFixedIntervalsNRD`,
`CIncrementalNodeRegistrationDecider`, `CICPCriteriaNRD`) left
`registration_max_distance`/`registration_max_angle` **uninitialized**, so a
decider used without `loadParams()` compared against garbage thresholds; the
documented defaults were dead code living only in `loadFromConfigFile()`.
(4) `CIncrementalNodeRegistrationDecider` did not compile at all if
instantiated: `getDescriptiveReport()` used an undefined `report_sep` and
`checkRegistrationCondition()` used an unqualified `INVALID_NODEID`; its 3D
registration check also printed `p1` twice via a leftover `std::cout` (now a
debug log). (5) `TUncertaintyPath::hasLowerUncertaintyThan()` was likewise
uninstantiable - it is `const` but called the non-`const` `getDeterminant()`;
the determinant cache is now `mutable` and the getter `const`.
(6) `CGlCanvasBase::setMousePos()`/`setMouseClicked()`/`updateLastPos()` and
`CGlCanvasBaseHeadless::renderError()` were declared in the public header but
never defined after the 3.x camera refactor - a link error for any caller, and
`CGlCanvasBaseHeadless` was unusable (its key function was missing). The two
click setters were dead (superseded by `COrbitCameraController`) and are gone;
`updateLastPos()` is restored and is now actually called from
`CWxGLCanvasBase`'s mouse handlers, which never recorded the pointer position,
so `CDisplayWindow3D::getLastMousePosition()` - and hence
`getLastMousePositionRay()` and all 3D picking - always returned pixel (0,0).
(7) `wxImage2MRPTImage()` passed `swapRedBlue = true`, a leftover from MRPT
2.x when `CImage` stored BGR; in 3.x both `CImage` and `wxImage` are RGB, so
every wxImage -> CImage conversion came back with red and blue swapped. The
matching dead `"BGR"` branch in `MRPTImage2wxImage()` was removed.
(8) `CWindowObserver::OnEvent()` compared the key modifiers against the magic
number `8192` with `==`, so Ctrl+C was missed whenever any other modifier
(e.g. Shift) was also held; it now masks against `mrpt::gui::MRPTKMOD_CONTROL`.
(9) `CNodeRegistrationDecider_impl.h` had `using namespace std;` at **global**
scope in a public header, leaking into every translation unit that included
any NRD; the definitions are now wrapped in their own namespace.

`mrpt_graphslam`'s remaining gap is `CEdgeCounter`'s visualization half and
`CWindowManager.h`, both of which need a live `CDisplayWindow3D` handed in
from outside the module. `mrpt_gui`'s is `CDisplayWindowGUI.cpp` (nanogui /
GLFW), `CQtGlCanvasBase.cpp` (Qt), `CAboutBox*`/`error_box.cpp` (modal dialogs
that would block a test run) and the rest of `mathplot.cpp`.

★ Coverage pass of 2026-09-07 on the four core algorithmic modules:
`mrpt_math` (88.8% -> 95.0%), `mrpt_maps` (86.5% -> 90.2%), `mrpt_obs`
(88.5% -> 91.6%) and `mrpt_slam` (90.2% -> 93.5%). All four are now at or
above the 90% goal.

The single most productive technique here was a ~30-line test helper that
writes an MRPT object frame (`[len|0x80][class name][version byte][payload]
[0x88]`) with an **arbitrary streaming version**, so the backwards-compatible
branches of `serializeFrom()` can be driven without shipping binary fixture
files. It lives as `tests/legacy_serialization.h` in both `mrpt_math` and
`mrpt_obs` (duplicated rather than shared, since modules are independent CMake
projects) and covers the legacy formats of `CPolygon` (v0/v1),
`CActionRobotMovement2D` (v0..v7, both the odometry and the streamed-PDF
paths), `CObservationStereoImages` (v0..v6), `CObservationImage` (v0..v4),
`CObservationBeaconRanges`, `CObservationIMU`, `CObservationGasSensors` and
`CObservation3DRangeScan` (v0..v10) - together several hundred lines that no
test had ever reached.

Real bugs found and fixed:

* `mrpt::math::TLine3D::TLine3D(const TLine2D&)` computed the base point of a
  *horizontal* 2D line (`A ~ 0`) as `-B/A`, dividing by the coefficient it had
  just tested for zero; it should be `-C/B`, as in `TLine2D::getAsPose2D()`.
  Every 2D->3D line conversion of a horizontal line yielded an infinite/NaN
  base point.
* `mrpt::math::getAngleBisector(TLine2D, TLine2D)`'s parallel-lines branch
  normalized the second line with `sqrt(A^2 + C^2)` instead of
  `sqrt(A^2 + B^2)`, and then added the two offsets without halving them. The
  two errors cancelled for a line whose normalization factor happened to equal
  `|C|`, which is exactly the case the pre-existing test used; for anything
  else the "bisector" was one of the two input lines.
* `mrpt::math::intersect(vector<T>, vector<U>, CSparseMatrixTemplate<O>&)`
  (geometry.h) iterated the inner loop up to `v1.size()` instead of
  `v2.size()`, reading out of bounds whenever the second set was smaller; its
  `std::vector<O>` sibling took the output container **by value**, so results
  never reached the caller. Both templates were uninstantiated dead code.
* `KDTreeCapable`'s radius-limited k-NN searches (`kdTreeNClosestPoint2D`,
  `kdTreeNClosestPoint3D`, `kdTreeNClosestPoint3DWithIdx`) trimmed the index
  and distance vectors to the number of points actually found but left the
  coordinate vectors at the requested `knn`, handing back stale entries; the
  `TPoint2D`/`TPoint3D` overloads then sized their output from those, so
  `pOut.size() != outDistSqr.size()`.
* `mrpt::math::CHistogram::createWithFixedWidth()` was declared as a
  non-static member, so the documented `CHistogram::createWithFixedWidth(...)`
  usage did not compile - which made `mrpt::math::CMonteCarlo`'s
  `getDistribution()` (its only caller) uninstantiable. Now `static`.
* `mrpt::math::KLD_Gaussians()` called the non-existent
  `inverse_LLt(out)` overload and `multiply_HCHt_scalar()` (row-vector form)
  on a column vector; it was likewise uninstantiable dead code.
* `CGasConcentrationGridMap2D::simulateAdvection()` indexes `m_stackedCov`,
  which only exists for the `mrKalmanApproximate` representation, so calling
  it on any other map type read past the end of an empty matrix and
  segfaulted. It now returns false with an error log instead.

Worth knowing for future tests here:

* `CObservationVelodyneScan`'s per-ray timestamps are derived from
  `CObservation::timestamp`, not from `getOriginalReceivedTimeStamp()` (which
  is left at `INVALID_TIMESTAMP` in a synthetically-built scan), so the
  `CPose3DInterpolator` fed to `generatePointCloudAlongSE3Trajectory()` must
  be built around the former, densely enough that every query has neighbors on
  both sides.
* Only the **auxiliary** particle filters go through
  `PF_SLAM_implementation_gatherActionsCheckBothActObs()`, which is what
  accumulates actions across steps; `pfStandardProposal` reads the action
  directly. And `PF_SLAM_implementation_doWeHaveValidObservations()` defaults
  to `true`, so an *empty* sensory frame is still "valid" - passing a null
  `sf` is the way to leave a movement accumulated for the next step.
* `CObservationGPS`'s `TIMECONV_IsALeapYear()` and
  `TIMECONV_GetNumberOfDaysInMonth()` are only ever called from the
  `seconds >= 60.0` rollover fix-up inside
  `TIMECONV_GetUTCTimeFromJulianDate()`. A sweep over ~2400 GPS weeks never
  gets the accumulated floating-point error above 59.999999 s, so that branch
  (and hence those two helpers) is unreachable in practice.
* `CRandomFieldGridMap2D`'s largest single block of untested code was
  `internal_clear()`'s `GMRF_use_occupancy_information` path, which builds the
  factor-graph prior by region growing over an occupancy gridmap. Pointing
  `insertionOptions.GMRF_gridmap_image_file` at the module's own
  `tests/map_pgm_32.pgm` fixture is enough to drive it end to end.
* `CGasConcentrationGridMap2D::build_Gaussian_Wind_Grid()` caches its look-up
  table in a file named after the grid parameters **in the current working
  directory**, and also writes a `simple_LUT.txt` debug dump. A test that
  wants to cover both the "generate + save" and the "found, load" branches
  must `chdir` into a scratch directory first (and restore it afterwards).

`mrpt_maps`' remaining gap is concentrated in
`CVoxelMapOccupancyBase.h`/`CVoxelMapBase.h` (voxel types other than the two
instantiated ones), `COccupancyGridMap2D_voronoi.cpp` and
`COccupancyGridMap3D.cpp`; `mrpt_slam`'s is `CMultiMetricMapPDF_RBPF.cpp`'s
landmark-map and no-odometry RO-SLAM branches of
`prediction_and_update_pfOptimalProposal()`.

### Weak areas, grouped by root cause

1. **Hardware drivers - `mrpt_hwdrivers` (29.0%)**: hard to unit-test since
   they talk to real serial ports/USB/GPS/LIDAR/cameras. The mockable-transport
   approach this entry used to call for was applied on 2026-09-07 (see § below)
   and works well for any driver that reads through an injectable `CStream`.
   What is still at 0% are the drivers that own their transport instead
   (`COpenNI2Generic`, `CKinect`, `CCameraSensor`, `CNTRIPClient`, `CLMS100eth`,
   `CSICKTim561Eth`, `CCANBusReader`, `CTaoboticsIMU`, ...); reaching them needs
   the same `bindIO()`/`bindStream()` treatment first. `mrpt_comms` cleared this
   bucket on 2026-08-29 (see » above): everything but `CInterfaceFTDI` turned
   out to be testable over loopback sockets and a pseudo-terminal.

2. **GUI/rendering — `mrpt_imgui` (0%) and the GUI-only files still left in
   `mrpt_gui` (40.6%)**: `CDisplayWindowGUI.cpp` (nanogui/GLFW),
   `CQtGlCanvasBase.cpp` (Qt), `CImGuiSceneView.cpp`, the modal dialogs in
   `CAboutBox*`/`error_box.cpp`, and the rest of `mathplot.cpp`. The
   `CDisplayWindow*`/`WxUtils`/`CWxGLCanvasBase`/`mathplot` bulk cleared this
   bucket on 2026-09-06 by running the tests under `xvfb-run` — see @ above;
   the same technique should work for the nanogui/Qt canvases.

3. **CLI apps — `mrpt_libapps_cli` (59.2% as of 2026-07-06, was 9.1%)**: some
   `rawlog-edit_*.cpp` paths remain untested. These are better suited to
   subprocess/golden-file integration tests (run the built binary against
   sample rawlogs, diff the output) than pure unit tests.

4. **Quick wins — pure-logic files at 0% with no hardware/GUI dependency**
   (highest-value gaps, ordinary unit tests would work immediately): none
   left as of 2026-09-06.
   (`mrpt_graphslam/src/CWindowObserver.cpp` and `mrpt_gui/src/CGlCanvasBase.cpp`
   cleared this bucket as of 2026-09-06 — see @ above.)
   (`mrpt_slam/src/slam/{CLandmarksMap,observations_overlap}.cpp` cleared this
   bucket as of 2026-08-31 — see × above.)
   (`mrpt_system/src/CFileSystemWatcher.cpp`, `mrpt_system/src/{progress,
   hyperlink,CObserver}.cpp`, `mrpt_io/src/{detect_compression,
   lazy_load_path}.cpp` and `mrpt_comms/src/{net_utils,CSerialPort}.cpp` all
   cleared this bucket as of 2026-08-29 — see » above.)
   (`mrpt_system/src/md5.cpp`, `mrpt_graphslam/src/{CEdgeCounter,
   TSlidingWindow}.cpp` and
   `mrpt_slam/src/slam/CRejectionSamplingRangeOnlyLocalization.cpp` all
   cleared this bucket as of 2026-08-28 — see ¤ above.)
   (`mrpt_img/src/CImage_loadXPM.cpp` cleared this bucket as of 2026-07-10, now ~90%;
   `mrpt_obs/src/gnss_messages_novatel.cpp` and `mrpt_obs/src/carmen_log_tools.cpp`
   also cleared as of 2026-07-10, now at 100% and 88.2% respectively;
   `mrpt_viz/src/PLY_import_export.cpp` and `mrpt_viz/src/COrbitCameraController.cpp`
   cleared as of 2026-08-02, now at 60.0% and 100% respectively.)

5. **Biggest single-file impact (most uncovered lines, worth prioritizing for
   raw percentage gains)**:
   `mrpt_nav/src/reactive/CAbstractPTGBasedReactive.cpp` (123, 82.1%),
   `mrpt_maps/include/mrpt/maps/CVoxelMapOccupancyBase.h` (92, 62.0%),
   `mrpt_maps/src/maps/CGenericPointsMap.cpp` (75, 84.5%),
   `mrpt_maps/src/maps/COccupancyGridMap2D_common.cpp` (72, 84.6%).
   (`mrpt_maps/src/maps/COccupancyGridMap2D_io.cpp` and `_simulate.cpp`
   cleared this bucket as of 2026-08-03.)
   (`mrpt_viz/src/CPolyhedron.cpp`, formerly the single biggest uncovered file
   in the whole repo at 1420 uncovered lines, cleared this bucket as of
   2026-08-02, now at 91.6%.)
   (`mrpt_obs/src/CObservation3DRangeScan.cpp` cleared this bucket as of
   2026-07-10, now at 89.1%; `mrpt_maps/src/maps/CRandomFieldGridMap2D.cpp`,
   `CPointsMap.cpp`, `CGasConcentrationGridMap2D.cpp`, `CColouredOctoMap.cpp`,
   `COctoMap.cpp`, and `COccupancyGridMap3D.cpp` all cleared it as of
   2026-07-17 — see § above.)

6. **Near-target modules (75-90%), smallest remaining gap to close first**:
   `mrpt_system` (83.4%; `CTimeLogger.cpp`, `COutputLogger.cpp` and
   `filesystem.cpp` are what is left), `mrpt_rtti` (85.8%, but see the dead
   registration queue noted in » above), `mrpt_io` (86.5%; `CPipe.cpp` needs
   child processes), `mrpt_comms` (75.8%; the rest is `CInterfaceFTDI`, which
   needs a real FTDI device), `mrpt_graphslam` (81.6%, needs a live
   `CDisplayWindow3D`), `mrpt_viz` (88.1%, see ◆ below).
   (`mrpt_math`, `mrpt_maps`, `mrpt_obs` and `mrpt_slam` cleared this bucket
   as of 2026-09-07 — see ★ above.)
   (`mrpt_serialization` cleared this bucket as of 2026-08-29, now at 96.6%;
   `mrpt_graphs` and `mrpt_random` cleared it as of 2026-07-06;
   `mrpt_bayes` and `mrpt_config` cleared it as of 2026-07-09, both now >96%;
   `mrpt_containers` cleared it as of 2026-07-11, now at 90.2%,
   remaining gaps being mostly defensive "should never happen" throws and
   libfyaml parser error paths that are difficult to trigger without a
   malformed internal parser state.)

◆ Coverage pass of 2026-09-07 (second pass of the day) on `mrpt_viz`
(70.1% -> 87.1%).

The two Assimp-based classes (`CAssimpModel` and `CAnimatedAssimpModel`,
~2000 lines) had no coverage at all. Their models are synthesized by the
tests themselves: plain geometry as a Wavefront OBJ plus an MTL and a PNG
texture written to a temp dir, and the rigged one as
`mrpt_viz/tests/skinned_model.gltf`, a hand-built 2-joint glTF 2.0 file with
one 3-keyframe rotation animation (a skinned, animated mesh is far easier to
author by hand in glTF than in any other Assimp-supported format; the binary
buffer is an inline base64 `data:` URI, so it stays a single text file).
Both test files guard themselves with `#if MRPT_HAS_ASSIMP`.

`RenderBuffers_unittest.cpp` is the durable guard for the biggest class of
defect found here: it asserts that every visual object actually fills the
CPU-side vertex/triangle buffers that `mrpt_opengl` later uploads to the GPU.
`CSetOfLines`, `CSimpleLine`, `CDisk` and `CFrustum` had lost their
buffer-filling code in the 2.x -> 3.x viz/opengl split and rendered *nothing*,
and `CPointCloud` had lost the color half of it (so `enableColorFromX/Y/Z()`
and `setGradientColors()` did nothing). The `mrpt_opengl` offscreen-rendering
tests did not catch any of it because their reference images had been
captured with the defect already present. When such an image legitimately
changes, regenerate it with
`MRPT_UPDATE_RENDER_REFERENCES=1 build/mrpt_opengl/bin/test_mrpt_opengl` under
`xvfb-run`; the switch and the shared `imageDiff()` live in
`mrpt_opengl/tests/render_reference.h`.

The other real bugs found and fixed:

* `CPointCloudColoured::PLY_export_get_vertex()` had its assignments
  reversed, writing the (uninitialized) output arguments *into* the cloud
  instead of reading a point out of it: saving a coloured cloud to PLY zeroed
  every point in memory and wrote a file of zeros. The pre-existing
  round-trip test passed because it compared the already-clobbered source
  against the equally empty reloaded copy - a reminder to assert against
  literals, not against the object that the code under test just touched.

* The PLY layer never requested `red`/`green`/`blue` from the file, so the
  importer's RGB branch was dead code and every colored PLY written by
  another tool lost its color; MRPT only ever wrote a grayscale `intensity`.
  Both directions now handle per-channel color (integer channels scaled from
  [0,255], float ones taken as [0,1]), `intensity` is still written for
  backwards compatibility, and RGB wins over it on load.

* `PlyProperty`'s constructor silently dropped its `is_list` argument, so the
  face element was written as `property int vertex_indices` instead of
  `property list uchar int vertex_indices`.

* `CPointCloud::setAllPointsFast()` and
  `CSetOfTriangles::insertTriangles(const Ptr&)` each took the object's write
  lock and then called a method that locks the same non-recursive
  `std::shared_mutex` again: both deadlocked on every call ("Resource
  deadlock avoided"). Worth grepping for this shape elsewhere.

* `CSetOfTriangles::updatePolygons()` assigned its scratch polygon to the
  output *inside* the per-vertex loop; `TPolygonWithPlane` fits a plane in
  its constructor and throws "points are aligned" for a partially filled
  triangle, so `traceRay()` failed on any mesh with a vertex at the origin.
  `getPolygons()` also wrote into its output vector without resizing it.

* `CAssimpModel::loadScene()` unconditionally OR'ed `aiProcess_GenSmoothNormals`
  into the Assimp flags, which Assimp rejects together with the
  `aiProcess_GenNormals` implied by its "fast" preset, so
  `LoadFlags::RealTimeFast` never loaded anything.

* `CAssimpModel::serializeFrom()` forwarded its own version number to
  `CSetOfObjects::serializeFrom()`, which only knows version 0, so no stream
  holding a current (v1) `CAssimpModel` could be read back; and
  `CAnimatedAssimpModel` was missing from `registerAllClasses()` entirely.
  The `SerializeTestOpenGL.WriteReadToMem` list in
  `mrpt_viz/tests/serializations_unittest.cpp` now covers *every* registered
  `mrpt::viz` class instead of a subset, which is what would have caught
  both.

* `CSetOfObjects::internalBoundingBoxLocal()` unioned its children's *local*
  boxes, ignoring each child's own pose within the set (`Viewport` already
  composed them the right way), so any composite object reported a box that
  did not contain its own geometry.

* `CMesh::adjustGridToImageAR()` and `CMeshFast::adjustGridToImageAR()` used
  width/height where height/width was meant, stretching the grid along the
  wrong axis.

* `CCamera::serializeTo()` wrote neither the base `CVisualObject` state nor
  the 6-DOF flag, so a camera placed with `set6DOFMode(true)` + `setPose()`
  lost both on save/load; `CVectorField3D` likewise dropped its module-based
  color-mapping settings. Both serialization versions were bumped, keeping
  the old readers.

A second pass on 2026-09-07 (87.1% -> 88.1%) closed the legacy
`serializeFrom()` version branches with the `legacy_serialization.h` technique
from ★ above. The viz copy of that header adds `writeLegacyRenderHeader()`,
since every viz class starts with `CVisualObject::writeToStreamRender()`, whose
format is versioned (0..4) *independently* of the class's own version - that
header is what makes these frames writable by hand at all. It found:

* `Viewport::serializeFrom()` read the "has image-view plane" flag added in v5
  without the version guard every other field there has, so any stream holding
  a viewport older than v5 - i.e. any `.3Dscene` written by a correspondingly
  old MRPT - desynchronized there and failed to load with an EOF error.
* The same function, and `COctoMapVoxels::serializeFrom()`, left the fields
  absent from an older stream (clip distances, viewport visibility, colormap)
  untouched instead of resetting them to their defaults, so loading an old
  file into a *reused* object silently kept the previous values. Worth
  checking for in any `serializeFrom()`: an `if (version >= N)` with no `else`.
* `CVisualObject::castShadows(bool doCast = true)` defaulted its argument,
  which made the no-argument `castShadows()` resolve to the *setter* on any
  non-const object - the getter was unreachable there, and a caller reading
  the property silently enabled shadow casting instead.
  `CVisualObject_unittest.cpp` had already worked around it with a const
  alias rather than fixing it. Grep for a getter/setter pair sharing a name
  where the setter's only argument is defaulted; this was the only one left.

The legacy branches of `CSphere`, `CBox`, `CSetOfLines`, `CPointCloud`,
`CArrow`, `CAxis` and `CPointCloudColoured` turned out to be correct; the
tests just pin them down.

What is left in `mrpt_viz`: `PLY_import_export.cpp` (the vendored Stanford
reader's per-type dispatch, only reachable with files using the less common
property types), and `CTextMessageCapable::regenerateGLobjects()`, which is
dead code: `mrpt_opengl`'s `CompiledViewport::renderTextOverlays()` builds the
text geometry directly from the label strings and never touches the `gl_text`
members.

Branch coverage lags line coverage everywhere (often by 15-30 points),
indicating error-handling and edge-case branches are the norm left untested
even in files with decent line coverage — prioritize adding failure-path
tests, not just more happy-path calls.

§ Coverage pass of 2026-09-07 (third pass of the day) on `mrpt_hwdrivers`
(13.9% -> 29.0%).

`mrpt_hwdrivers/tests/mock_stream.h` is the lever for all of it: a `CStream`
that records everything the driver writes and replays scripted answers keyed
on the command received, which is what request/response sensor protocols need.
Any driver reachable through `C2DRangeFinderAbstract::bindIO()` or
`CGPSInterface::bindStream()` becomes testable with it; the covered ones are
`C2DRangeFinderAbstract` (0% -> 99%), `CHokuyoURG` (0% -> 62%, the whole
SCIP2.0 handshake and scan decoder), `CSickLaserSerial` (0% -> 33%) and
`CGPSInterface` (19% -> 47%).

The pcap half of `CVelodyneScanner` (0% -> 40%) was dead code, not untested
code, and it took four separate fixes to revive: there was no `FindPCAP.cmake`
in the repo so `find_package(PCAP)` always failed; `MRPT_HAS_LIBPCAP` was never
emitted into `config.h`; `CVelodyneScanner.cpp` never included that `config.h`;
and the unit test guarded on `MRPT_HAS_TINYXML2` without including
`mrpt/obs/config.h`. Both Velodyne tests had therefore been compiling to
nothing on every platform, sample `.pcap` datasets and all.

Warning for anyone extending this: several `MRPT_HAS_*` macros that hwdrivers
sources still guard on are never defined anywhere in the 3.x build
(`MRPT_HAS_OPENCV`, `MRPT_HAS_LIBDC1394_2`, `MRPT_HAS_ROBOPEAK_LIDAR`,
`MRPT_HAS_NIDAQMX*`, `MRPT_HAS_PGR_FLYCAPTURE2`, `MRPT_HAS_KINECT_CL_NUI`), so
those code paths are unconditionally compiled out. Do not "fix" one by adding
the define alone: enabling `MRPT_HAS_LIBDC1394_2` was tried here and
`CImageGrabber_dc1394.cpp` no longer compiles against current `mrpt::img`
(6 errors), having rotted unnoticed for as long as the macro was missing.
Several sources also fail to include `mrpt/hwdrivers/config.h` at all
(`COpenNI2*`, `CNationalInstrumentsDAQ`, `CImageGrabber_FlyCapture2`,
`CPhidgetInterfaceKitProximitySensors`), so even their defined macros read 0.

Seven defects were found by the new tests, five of them crashes:

 - `C2DRangeFinderAbstract::getObservation()` could never return anything:
   `m_lastObservation`/`m_lastObservationIsNew`/`m_hardwareError` were read but
   written nowhere in the class.
 - `CHokuyoURG::parseResponse()` validated the device status code only on
   replies carrying data, so an error answer to a status-only command (`BM`,
   `QT`, `CR`, ...) counted as success.
 - `CGPSInterface::OnConnectionShutdown()`, called from the destructor,
   dereferenced a null stream when shutdown commands were configured but
   nothing was ever opened.
 - `CGPSInterface` setup commands were sent only from the serial-open path, so
   with an externally bound stream they were dropped while the matching
   shutdown commands were still sent.
 - `~CIbeoLuxETH()` joined a never-started thread, `~CRoboPeakLidar()` called a
   `turnOff()` that throws without the SDK, `~CImpinjRFID()` wrote to a null
   socket and `~CGyroKVHDSP3000()` closed a null serial port -- each aborting
   the process when a sensor was created by the factory and destroyed before
   `initialize()`, which is precisely what `rawlog-grabber` does between
   `createSensor()` and configuring it. `CGenericSensor_unittest.cpp` walks
   every registered driver to keep that path honest.

`CSickLaserSerial` advertised `bindIO()` but its frame reader, ACK waiter and
command sender each `dynamic_cast`ed to `CSerialPort` and asserted on it, so
any other stream aborted on the first read. Those four functions only ever call
`Read()`/`Write()`, so they now use the bound stream; `open`/`setConfig`/
`purgeBuffers` are genuinely serial-specific and were left alone.

## 11. mrpt_nav API modernization + TP-Space math fixes (2026-09-08)

An API/mathematics pass over `mrpt_nav`, separate from the coverage pass of ¶.
Nothing here is a coverage-driven change; the test count went 246 -> 251.

**Out-param APIs replaced by `std::optional`** (old signature kept as a
`[[deprecated]]` inline shim unless noted), matching what `inverseMap_WS2TP()`
already did:

* `CParameterizedTrajectoryGenerator::getPathStepForDist(k, dist)`. The old
  3-arg form also wrote the *last* path step into `out_step` when it returned
  false, and two call sites silently relied on that; the explicit
  `getPathStepForDistClamped()` now covers it.
* `nav_plan_geometry_utils`: `collision_free_dist_{segment,arc}_circ_robot()`.
* `PlannerSimple2D::computePath()`.

**Other API changes**: `ClearanceDiagram::getClearance()`'s `bool
integrate_over_path` became `enum class ClearanceQuery` (see below);
`CPTG_Holo_Blend::PATH_TIME_STEP` (a mutable global) became the per-instance
`path_time_step` config key + `setPathTimeStep()`, and `eps` became
`EPSILON`, both no-shim breaks; `setScorePriorty()` -> `setScorePriority()`;
`updateClearancePost()` (a documented no-op since 2017) and
`CAbstractHolonomicReactiveMethod::Create()` (declared in the header but
**never defined anywhere** -- calling it was a link error) were deleted.

**Real bugs fixed** -- each has a regression test that fails without the fix:

1. `ClearanceDiagram::getClearance()`'s two modes were **swapped** relative to
   its own docs, to every call site's comment, and to the ptg-configurator's
   UI label; `bool=false` averaged over the path and `bool=true` returned the
   spot value. The reactive navigator's `clearance` / `clearance_path` score
   factors were therefore each other's values (only visible with
   `evaluate_clearance=true`, off by default).
2. `initClearanceDiagram()` keyed the clearance samples by the raw path
   distance **in meters** while every consumer treats those keys as normalized
   [0,1] TPS distances (`getClearance()` queries, the `dist_over_path > 0.5`
   collision heuristic, and `CAbstractPTGBasedReactive`'s own
   `dist_eucl_min` producer, which builds the same map keyed `i/num_steps`).
3. Same function sampled steps `0, incr, 2*incr...` while
   `evalClearanceSingleObstacle()` evaluates `incr, 2*incr, ...`, so every
   clearance value was filed under a distance shorter than the one it was
   measured at. Both loops now use the same steps.
4. `CHolonomicVFF::navigate()` assigned `desiredSpeed` **inside**
   `if (m_enableApproachTargetSlowDown)`, so with the slow-down disabled it
   returned speed 0 and the robot never moved. ND/FullEval had it right.
5. `CHolonomicFullEval` used `ni.targets.front()` for the approach slow-down;
   `NavInput` documents the *last* target as the highest-priority one.
6. `CPTG_Holo_Blend` used `V_MAX` for the post-ramp cruise speed in
   `getPathDist()`, `getPathStepForDist()` and `updateTPObstacleSingle()`,
   but `getPathPose()` advances at the direction-dependent
   `internal_get_v(dir)`. With an `expr_V` set, poses and distances disagreed.
   `inverseMap_WS2TP()` likewise pinned `T_ramp = T_ramp_max` instead of
   `internal_get_T_ramp(alpha)`; it is now re-evaluated per Newton iteration
   (the Jacobian ignores dT/dalpha, which only costs iterations, not accuracy,
   since the residual is exact).
7. `CAbstractPTGBasedReactive::calc_move_candidate_scores()` fed the
   *normalized* collision-free distance to `getPathStepForDist()`, which takes
   pseudometers (the ETA factor 300 lines below does `d * ref_dist` for the
   very same call). The "end of trajectory" behind `robpose_*`,
   `dist_eucl_final` and the target slow-down check was therefore read
   `ref_distance` times too early along the path -- 0.74 m instead of 4 m in
   the regression test.
8. `collision_free_dist_arc_circ_robot()`'s closed form divided by `o.x`, so
   **any** obstacle on the turn-center axis returned NaN. Rewritten as a
   two-circle intersection: agrees with the old formula to 1.7e-11 over 21k
   random collision cases, and it now returns 0 when the robot starts already
   in collision instead of the *exit* distance.

**Math model**: `calc_trans_distance_t_below_Tramp_abc_numeric()` went from a
15-interval trapezoidal rule to 16-interval Simpson (same number of function
evaluations, ~25x lower mean relative error), plus an exact branch for the
degenerate `b^2-4ac ~= 0` case where the integrand is `sqrt(a)*|t-r|`.

`CPTG_Holo_Blend::m_pathStepCountCache` is expressed in path time steps, so
every write to `m_pathTimeStep` now goes through `setPathTimeStep()`, which
clears it and rejects non-finite/non-positive values (including from a stream).

New console example `mrpt_examples_cpp/nav_ptg_tpspace` walks the whole
WS -> TP-Space -> velocity-command round trip headlessly.
