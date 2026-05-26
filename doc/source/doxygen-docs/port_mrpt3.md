\page porting_mrpt3 Porting code from MRPT 2.x to MRPT 3.0

# Porting guide: MRPT 2.x to MRPT 3.0

This document describes all breaking changes in MRPT 3.0 and how to migrate
existing code that was written against MRPT 2.x.

## Why MRPT 3.0? — Advantages of the refactoring

- **Modular colcon-based build**: Each MRPT module (`mrpt_core`, `mrpt_math`,
  `mrpt_poses`, `mrpt_viz`, …) is an independent colcon package with its own
  `CMakeLists.txt`. No ROS dependency is required — plain colcon is used as the
  meta-build tool, enabling faster incremental builds and independent
  versioning.
- **Clean namespace separation**: The scene-graph / 3-D visualization API now
  lives in `mrpt::viz`, while `mrpt::opengl` is reserved for low-level GPU
  rendering internals (shaders, FBOs, textures). This makes the public API
  surface smaller and easier to learn.
- **Modern C++ API improvements**: output-reference parameters replaced by
  `std::optional` returns, raw coordinate pairs replaced by `TPixelCoord`,
  integer channel arguments replaced by `TImageChannels` enum, and more.
- **Removed legacy / deprecated code**: octree rendering settings, sparse
  matrices, old feature matching, `CColouredPointsMap`, and other rarely-used
  APIs have been removed, reducing maintenance burden and binary size.

---

## 1. Build system changes

### CMake package names

The pattern `mrpt-<name>` is now `mrpt_<name>` (hyphens replaced by underscores):

- `find_package(mrpt-poses)` becomes `find_package(mrpt_poses)`
- `find_package(mrpt-gui)` becomes `find_package(mrpt_gui)`
- `find_package(mrpt-slam)` becomes `find_package(mrpt_slam)`

### CMake target names

The pattern `mrpt::<name>` is now `mrpt::mrpt_<name>`:

- `mrpt::poses` becomes `mrpt::mrpt_poses`
- `mrpt::gui` becomes `mrpt::mrpt_gui`
- `mrpt::slam` becomes `mrpt::mrpt_slam`

### Example CMakeLists.txt (MRPT 3.0)

```cmake
cmake_minimum_required(VERSION 3.16)
project(my_app)

find_package(mrpt_common REQUIRED)   # provides mrpt_add_executable()
find_package(mrpt_gui REQUIRED)
find_package(mrpt_maps REQUIRED)

mrpt_add_executable(
  TARGET my_app
  SOURCES main.cpp
  LINK_LIBRARIES mrpt::mrpt_gui mrpt::mrpt_maps
)
```

### Header for examples base directory

MRPT 2.x provided `<mrpt/examples_config.h>` which defined
`MRPT_EXAMPLES_BASE_DIRECTORY`. In MRPT 3.0 this header no longer exists.
Instead, define it as a compile definition in your CMakeLists.txt:

```cmake
add_compile_definitions(
    MRPT_EXAMPLES_BASE_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/")
```

---

## 2. Namespace and header renames

### `mrpt::opengl` → `mrpt::viz`

The entire scene-graph API (3-D objects, scenes, viewports, cameras, stock
objects) has moved from `mrpt::opengl` to `mrpt::viz`.

- `#include <mrpt/opengl/CPointCloud.h>` becomes `#include <mrpt/viz/CPointCloud.h>`
- `#include <mrpt/opengl/Scene.h>` becomes `#include <mrpt/viz/Scene.h>`
- `#include <mrpt/opengl/stock_objects.h>` becomes `#include <mrpt/viz/stock_objects.h>`
- `mrpt::opengl::CPointCloud` becomes `mrpt::viz::CPointCloud`
- `mrpt::opengl::Scene` becomes `mrpt::viz::Scene`
- `opengl::stock_objects::CornerXYZ()` becomes `viz::stock_objects::CornerXYZ()`

**What stays in `mrpt::opengl`**: Only low-level GPU rendering internals:
- `mrpt::opengl::CFBORender` — `#include <mrpt/opengl/CFBORender.h>`
- `mrpt::opengl::Shader` — `#include <mrpt/opengl/Shader.h>`
- `mrpt::opengl::Buffer` — `#include <mrpt/opengl/Buffer.h>`
- `mrpt::opengl::Texture` — `#include <mrpt/opengl/Texture.h>`
- `mrpt::opengl::DefaultShaders` — `#include <mrpt/opengl/DefaultShaders.h>`
- `#include <mrpt/opengl/opengl_api.h>`

### Catch-all header removed

The header `#include <mrpt/opengl.h>` no longer exists. Replace it with
individual `#include <mrpt/viz/...>` headers for each class you use.

### Class renames

- `COpenGLScene` becomes `mrpt::viz::Scene`
- `COpenGLViewport` becomes `mrpt::viz::Viewport`
- `CRenderizable` becomes `mrpt::viz::CVisualObject`

---

## 3. `TPixelCoord` for drawing functions

All `CCanvas` / `CImage` drawing methods that previously took separate
`int x, int y` parameters now take a single `TPixelCoord{x, y}` struct.

```cpp
// MRPT 2.x:
img.textOut(10, 20, "Hello", TColor::white());
img.line(x0, y0, x1, y1, TColor::red());
img.filledRectangle(0, 0, w, h, TColor::black());

// MRPT 3.0:
img.textOut({10, 20}, "Hello", TColor::white());
img.line({x0, y0}, {x1, y1}, TColor::red());
img.filledRectangle({0, 0}, {w, h}, TColor::black());
```

This also applies to `CDisplayWindow3D::get3DRayForPixelCoord()`:

```cpp
// MRPT 2.x:
viewport->get3DRayForPixelCoord(x, y, ray);

// MRPT 3.0:
auto ray_opt = viewport->get3DRayForPixelCoord({x, y});
if (ray_opt.has_value()) { auto& ray = ray_opt.value(); ... }
```

---

## 4. `std::optional` return types

Several methods that previously returned `bool` and wrote to an output
reference now return `std::optional<T>`:

```cpp
// MRPT 2.x:
mrpt::math::TLine3D ray;
bool ok = vp->get3DRayForPixelCoord(x, y, ray);

// MRPT 3.0:
auto ray = vp->get3DRayForPixelCoord({x, y});
if (ray.has_value()) { /* use *ray */ }
```

---

## 5. `CImage::loadFromFile` channel argument

The second argument to `CImage::loadFromFile()` changed from `int` to the
`TImageChannels` enum:

```cpp
// MRPT 2.x:
img.loadFromFile("file.png", 0);   // 0 = grayscale

// MRPT 3.0:
img.loadFromFile("file.png", mrpt::img::CH_GRAY);
// Other values: CH_RGB, CH_RGBA, CH_AS_IS
```

---

## 6. `CImage::operator()` removed

The pixel-access `operator()` has been removed. Use `ptr<T>()` instead:

```cpp
// MRPT 2.x:
*img(x, y) = 128;

// MRPT 3.0:
*img.ptr<uint8_t>(x, y) = 128;
```

---

## 7. `CDisplayWindow3D` changes

The camera-control methods on `CDisplayWindow3D` remain, but the
`CFBORender::getCamera(scene)` method has been replaced:

```cpp
// MRPT 2.x:
CCamera& cam = render.getCamera(scene);
cam.setZoomDistance(50);

// MRPT 3.0:
CCamera cam;
cam.setZoomDistance(50);
render.setCamera(cam);
// To read back: render.getCameraOverride()
```

---

## 8. `CArrow` constructor

`CArrow` now takes `TPoint3Df` arguments instead of 6 separate floats:

```cpp
// MRPT 2.x:
auto arrow = CArrow::Create(x0, y0, z0, x1, y1, z1, headRatio, smallR, largeR);

// MRPT 3.0:
auto arrow = CArrow::Create({x0, y0, z0}, {x1, y1, z1}, headRatio, smallR, largeR);
```

---

## 9. Removed APIs

The following APIs have been removed in MRPT 3.0 with no direct replacement:

- `mrpt::vision` — Removed (except a few classes moved to mrpt::img, see next point); will move computer vision features to a new package `mola_vision`.
- `mrpt::global_settings::OCTREE_RENDER_MAX_DENSITY_POINTS_PER_SQPIXEL` — Octree rendering settings removed.
- `mrpt::global_settings::OCTREE_RENDER_MAX_POINTS_PER_NODE` — Octree rendering settings removed.
- `CPointCloud::octree_get_graphics_boundingboxes()` — Octree API removed.
- `CPointCloud::getActuallyRendered()` — Octree API removed.
- `CPointCloud::octree_get_visible_nodes()` — Octree API removed.
- `CPointCloud::octree_get_node_count()` — Octree API removed.
- `mrpt::maps::CColouredPointsMap` — Use `CSimplePointsMap` or `CPointsMapXYZI`.
- `mrpt::math::CSparseMatrix` — Removed; use Eigen sparse matrices.
- `CLandmarksMap::insertionOptions` / `likelihoodOptions` — Removed.
- `#include <mrpt/opengl.h>` — Catch-all header removed.
- `#include <mrpt/examples_config.h>` — Use CMake compile definition instead.
- `#include <mrpt/config.h>` — Use module-specific config headers (e.g. from mrpt_opengl).

---

## 10. `TLightParameters` multi-light API

`TLightParameters` no longer stores a single directional light via flat fields
(`direction`, `diffuse`, `specular`, `color`). Instead it holds a `std::vector<TLight> lights`
supporting up to 8 simultaneous lights of three types: `TLightType::Directional`,
`TLightType::Point`, and `TLightType::Spot`.

**Before (MRPT 2.x):**
```cpp
auto& lp = viewport->lightParameters();
lp.direction = {-0.4f, -0.4f, -0.8f};
lp.diffuse   = 0.8f;
lp.specular  = 0.95f;
lp.color     = {1.0f, 1.0f, 1.0f};
```

**After (MRPT 3.0):**
```cpp
auto& lp = viewport->lightParameters();
// Access the default directional light:
lp.lights[0].direction = {-0.4f, -0.4f, -0.8f};
lp.lights[0].diffuse   = 0.8f;
lp.lights[0].specular  = 0.95f;
lp.lights[0].color     = {1.0f, 1.0f, 1.0f};

// Add a point light:
lp.lights.push_back(mrpt::viz::TLight::PointLight(
    {5.0f, 0.0f, 3.0f},  // position
    {1.0f, 0.9f, 0.8f}   // color
));
```

The global `ambient` field remains on `TLightParameters`. Shadow mapping
continues to apply only to the first directional light.

---

## 11. Moved classes/functions

- `mrpt::vision::CVideoFileWriter.h` becomes `mrpt::img::CVideoFileWriter.h`. It no longer supports MP4, only simpler AVI formats, but it is selfcontained without external libraries.
- `mrpt::hwdrivers::prepareVideoSourceFromUserSelection()` becomes `mrpt::apps::prepareVideoSourceFromUserSelection()` (header: `<mrpt/apps_gui/CameraSelectionGUI.h>`).

When using this function, add `mrpt_libapps_gui` to your CMake dependencies.

---

## 12. Return-by-value API modernization

### `CObservation::getSensorPose()`

The primary API is now return-by-value:

```cpp
// MRPT 2.x / deprecated:
mrpt::poses::CPose3D pose;
obs->getSensorPose(pose);

// MRPT 3.0:
mrpt::poses::CPose3D pose = obs->getSensorPose();
```

All concrete observation classes (`CObservation2DRangeScan`, `CObservation3DRangeScan`, etc.)
now override `CPose3D getSensorPose() const` instead of `void getSensorPose(CPose3D&) const`.
The out-param overload is kept as a deprecated non-virtual wrapper for backwards compatibility.

### Pose homogeneous matrix accessors

`getHomogeneousMatrix()` now returns by value on all pose types:

```cpp
// MRPT 2.x / deprecated:
mrpt::math::CMatrixDouble44 HM;
pose.getHomogeneousMatrix(HM);

// MRPT 3.0:
auto HM = pose.getHomogeneousMatrix();
```

Same pattern for `CPose3D::getRotationMatrix()`, `CPose2D::getHomogeneousMatrix()`,
`CPose3DQuat::getHomogeneousMatrix()`.

### `CPoseRandomSampler` covariance accessors

```cpp
// MRPT 2.x / deprecated:
mrpt::math::CMatrixDouble33 cov;
sampler.getOriginalPDFCov2D(cov);

// MRPT 3.0:
auto cov = sampler.getOriginalPDFCov2D();
auto cov6 = sampler.getOriginalPDFCov3D();
```

### `CPose3DPDFGaussian::getCovSubmatrix2D()`

```cpp
// MRPT 2.x / deprecated:
mrpt::math::CMatrixDouble out;
pdfGauss.getCovSubmatrix2D(out);

// MRPT 3.0:
auto out = pdfGauss.getCovSubmatrix2D();
```

### `CAngularObservationMesh` accessors

```cpp
// MRPT 2.x / deprecated:
mrpt::viz::CSetOfTriangles::Ptr tris;
mesh.generateSetOfTriangles(tris);
mrpt::viz::CSetOfLines::Ptr rays;
mesh.getTracedRays(rays);
mesh.getUntracedRays(rays, dist);

// MRPT 3.0:
auto tris = mesh.generateSetOfTriangles();
auto rays = mesh.getTracedRays();
auto rays2 = mesh.getUntracedRays(dist);
```

### `CGasConcentrationGridMap2D::getWindAs3DObject()`

```cpp
// MRPT 2.x / deprecated:
mrpt::viz::CSetOfObjects::Ptr obj;
gasMap.getWindAs3DObject(obj);

// MRPT 3.0:
auto obj = gasMap.getWindAs3DObject();
```

### Hardware driver grabbers

`getObservation()` is deprecated in favour of `grabFrame()` which returns `std::optional<T>`:

```cpp
// MRPT 2.x / deprecated:
mrpt::obs::CObservationImage obs;
bool ok = camera.getObservation(obs);

// MRPT 3.0:
auto obs_opt = camera.grabFrame();
if (obs_opt) { /* use *obs_opt */ }
```

Affected classes: `CImageGrabber_OpenCV`, `CImageGrabber_dc1394`, `CMyntEyeCamera`,
`CFFMPEG_InputStream` (uses `grabFrame()` returning `std::optional<CImage>`),
`CEnoseModular`, `CWirelessPower`.

### `circular_buffer::pop()`

```cpp
// MRPT 2.x / deprecated:
T val;
buf.pop(val);

// MRPT 3.0:
T val = buf.pop();
```

---

## 13. Scoped enum replacements

### `CRobot2NavInterface::stop()` — `StopType` enum

```cpp
// MRPT 2.x / deprecated:
iface.stop(true);   // isEmergencyStop=true

// MRPT 3.0:
iface.stop(mrpt::nav::StopType::Emergency);
iface.stop(mrpt::nav::StopType::Normal);
```

### `CPointsMap::savePCDFile()` — `PCDFormat` enum

```cpp
// MRPT 2.x / deprecated:
map.savePCDFile("out.pcd", true);   // binary=true

// MRPT 3.0:
map.savePCDFile("out.pcd", mrpt::maps::PCDFormat::Binary);
map.savePCDFile("out.pcd", mrpt::maps::PCDFormat::ASCII);
```

### `vectorToTextFile()` — `VectorTextFileOptions` struct

```cpp
// MRPT 2.x / deprecated:
mrpt::math::vectorToTextFile(v, "file.txt", false, true);  // append=false, byRows=true

// MRPT 3.0:
mrpt::math::VectorTextFileOptions opts;
opts.append = false;
opts.byRows = true;
mrpt::math::vectorToTextFile(v, "file.txt", opts);
```

### `CRawlog::TEntryType` — scoped enum

```cpp
// MRPT 2.x:
if (entry_type == CRawlog::etSensoryFrame) { ... }

// MRPT 3.0:
if (entry_type == CRawlog::TEntryType::etSensoryFrame) { ... }
```

---

## 14. Step-by-step migration checklist

1. **CMakeLists.txt**: Replace `mrpt-foo` → `mrpt_foo` in `find_package` calls,
   and `mrpt::foo` → `mrpt::mrpt_foo` in `target_link_libraries`.
   Optionally, add `find_package(mrpt_common REQUIRED)` and use `mrpt_add_executable()`.

2. **Headers**: Replace `#include <mrpt/opengl/Foo.h>` →
   `#include <mrpt/viz/Foo.h>` for all scene-graph classes. Keep
   `mrpt/opengl/` only for `CFBORender`, `Shader`, `Buffer`, `Texture`,
   `DefaultShaders`, `opengl_api.h`.

3. **Namespaces**: Replace `mrpt::opengl::` → `mrpt::viz::` and
   `opengl::` → `viz::` (including `using namespace` declarations).

4. **Class names**: `COpenGLScene` → `Scene`, `COpenGLViewport` → `Viewport`,
   `CRenderizable` → `CVisualObject`.

5. **Drawing functions**: Convert `textOut(x, y, ...)`, `line(x0, y0, x1, y1, ...)`,
   `filledRectangle(x0, y0, x1, y1, ...)` to use `TPixelCoord` / brace-init.

6. **`CImage::loadFromFile`**: Replace integer channel args with `TImageChannels` enum.

7. **`CFBORender`**: Replace `getCamera(scene)` with `setCamera()` /
   `getCameraOverride()`.

8. **Remove calls** to deleted `global_settings::OCTREE_*`, octree query
   methods, `matchFeatures`, and other removed APIs.

9. **`prepareVideoSourceFromUserSelection`**: Move from `mrpt::hwdrivers` to
   `mrpt::apps`, add `mrpt_libapps_gui` dependency.

10. **Build and iterate**: Use `colcon build --packages-up-to mrpt_<module>`
    to build incrementally and fix remaining issues.

---

## 15. `CParticleFilter` enum class migration

`TParticleFilterAlgorithm` and `TParticleResamplingAlgorithm` are now
`enum class` with new PascalCase value names. The old names remain as
`static constexpr` aliases for source compatibility, but prefer the new names:

**Algorithm enum:**

| Old name (still valid) | New name |
|---|---|
| `CParticleFilter::pfStandardProposal` | `CParticleFilter::TParticleFilterAlgorithm::StandardProposal` |
| `CParticleFilter::pfAuxiliaryPFStandard` | `CParticleFilter::TParticleFilterAlgorithm::AuxiliaryPFStandard` |
| `CParticleFilter::pfOptimalProposal` | `CParticleFilter::TParticleFilterAlgorithm::OptimalProposal` |
| `CParticleFilter::pfAuxiliaryPFOptimal` | `CParticleFilter::TParticleFilterAlgorithm::AuxiliaryPFOptimal` |

**Resampling enum:**

| Old name (still valid) | New name |
|---|---|
| `CParticleFilter::prMultinomial` | `CParticleFilter::TParticleResamplingAlgorithm::Multinomial` |
| `CParticleFilter::prResidual` | `CParticleFilter::TParticleResamplingAlgorithm::Residual` |
| `CParticleFilter::prStratified` | `CParticleFilter::TParticleResamplingAlgorithm::Stratified` |
| `CParticleFilter::prSystematic` | `CParticleFilter::TParticleResamplingAlgorithm::Systematic` |

---

## 16. `CRejectionSamplingCapable::rejectionSampling` return by value

`rejectionSampling` previously wrote into an output `std::vector<TParticle>&`
parameter. It now returns the vector:

```cpp
// MRPT 2.x / deprecated:
std::vector<TParticle> samples;
sampler.rejectionSampling(200, samples, 1000);

// MRPT 3.0:
auto samples = sampler.rejectionSampling(200, 1000);
```

---

## 17. `CParticleFilterCapable::TParticleProbabilityEvaluator` is now `std::function`

The callback type used in `prepareFastDrawSample` changed from a raw function
pointer to `std::function`. Existing code that passes a plain function pointer
or a static member function pointer continues to work unchanged. Lambdas and
functors can now be passed directly without wrapping.

---

## 18. `CKalmanFilterCapable`: `OnSubstractObservationVectors` renamed

The virtual method `OnSubstractObservationVectors` ("substract" is a misspelling)
has been renamed to `OnSubtractObservationVectors`. The old name is kept as a
deprecated trampoline:

```cpp
// MRPT 2.x (still compiles with a deprecation warning):
void MyKF::OnSubstractObservationVectors(KFArray_OBS& A, const KFArray_OBS& B) const override { ... }

// MRPT 3.0:
void MyKF::OnSubtractObservationVectors(KFArray_OBS& A, const KFArray_OBS& B) const override { ... }
```

---

## 19. Porting ROS 2 nodes from `mrpt_ros` (MRPT 2.x) to native MRPT 3.x

Starting with MRPT 3.0, the upstream [MRPT/mrpt](https://github.com/MRPT/mrpt) repository
is itself a colcon-friendly multi-package workspace, so the `mrpt_ros` wrapper is no longer
needed. Downstream ROS 2 packages should depend directly on the native `mrpt_<module>`
packages shipped by MRPT 3.x.

> **Note:** The legacy `mrpt_ros` packages remain released on the ROS build farm for
> already-released ROS distros, but no new features will land there. New downstream
> releases should target MRPT 3.x directly.

### 19.1 `package.xml` — dependency renaming

Replace every `<depend>mrpt_lib*</depend>` with the fine-grained module(s) actually used:

| `mrpt_ros` (2.x) | MRPT 3.x native packages |
|---|---|
| `mrpt_libbase`      | `mrpt_io`, `mrpt_serialization`, `mrpt_system`, `mrpt_rtti`, `mrpt_containers`, `mrpt_typemeta`, `mrpt_random`, `mrpt_config`, `mrpt_expr` |
| `mrpt_libmath`      | `mrpt_math` |
| `mrpt_libposes`     | `mrpt_poses` (+ `mrpt_tfest`, `mrpt_bayes` if used) |
| `mrpt_libobs`       | `mrpt_obs` (+ `mrpt_topography` if used) |
| `mrpt_libmaps`      | `mrpt_maps` (+ `mrpt_graphs` if used) |
| `mrpt_libopengl`    | `mrpt_opengl` (+ `mrpt_img`) |
| `mrpt_libgui`       | `mrpt_gui` |
| `mrpt_libnav`       | `mrpt_nav` (+ `mrpt_kinematics`) |
| `mrpt_libslam`      | `mrpt_slam` (+ `mrpt_vision`) |
| `mrpt_libhwdrivers` | `mrpt_hwdrivers` (+ `mrpt_comms`) |
| `mrpt_libapps`      | `mrpt_libapps_cli`, `mrpt_libapps_gui` |
| `mrpt_apps`         | `mrpt_apps_cli` / `mrpt_apps_gui` |
| `mrpt_libtclap`     | **Removed** — migrate to [CLI11](https://github.com/CLIUtils/CLI11) (`cli11-dev` / rosdep key `cli11`), `argparse`, or `cxxopts` |
| `python_mrpt`       | **Removed** — each module now has its own pybind11 bindings |

Example `package.xml` diff:

```diff
 <package format="3">
   <name>my_ros_node</name>
-  <depend>mrpt_libmath</depend>
-  <depend>mrpt_libposes</depend>
-  <depend>mrpt_libmaps</depend>
-  <depend>mrpt_libtclap</depend>
+  <depend>mrpt_math</depend>
+  <depend>mrpt_poses</depend>
+  <depend>mrpt_maps</depend>
+  <depend>cli11</depend>
 </package>
```

### 19.2 `CMakeLists.txt` — `find_package` and target renaming

The CMake package names now use underscores and the exported targets are
`mrpt::mrpt_<module>` (not `mrpt::<module>`):

```diff
-find_package(mrpt-math   REQUIRED)
-find_package(mrpt-poses  REQUIRED)
-find_package(mrpt-maps   REQUIRED)
-find_package(mrpt-tclap  REQUIRED)
+find_package(mrpt_math   REQUIRED)
+find_package(mrpt_poses  REQUIRED)
+find_package(mrpt_maps   REQUIRED)
+find_package(CLI11       REQUIRED)

 add_executable(my_node src/main.cpp)
 target_link_libraries(my_node PRIVATE
-  mrpt::math
-  mrpt::poses
-  mrpt::maps
-  mrpt::tclap
+  mrpt::mrpt_math
+  mrpt::mrpt_poses
+  mrpt::mrpt_maps
+  CLI11::CLI11
 )
```

The mechanical rules are:
- `find_package(mrpt-<X>)` → `find_package(mrpt_<X>)`
- `mrpt::<X>` → `mrpt::mrpt_<X>`

### 19.3 Dropped: `mrpt-tclap` / `mrpt_libtclap`

MRPT 3.x no longer vendors TCLAP. Code using
`#include <mrpt/3rdparty/tclap/CmdLine.h>` must migrate to an external CLI
library. The recommended replacement is **CLI11** (single-header, packaged as
`libcli11-dev` on Ubuntu, rosdep key `cli11`).

Minimal port sketch (TCLAP → CLI11):

```cpp
// Before (TCLAP via the removed mrpt_libtclap):
#include <mrpt/3rdparty/tclap/CmdLine.h>
TCLAP::CmdLine cmd("my tool", ' ', "1.0");
TCLAP::ValueArg<std::string> arg_in("i", "input", "input file", true, "", "file", cmd);
cmd.parse(argc, argv);
const std::string in = arg_in.getValue();

// After (CLI11):
#include <CLI/CLI.hpp>
CLI::App app{"my tool"};
std::string in;
app.add_option("-i,--input", in, "input file")->required();
CLI11_PARSE(app, argc, argv);
```

---

## 20. `TKF_options::use_joseph_form` (new option)

A new boolean `TKF_options::use_joseph_form` (default `true`) controls whether
the kfEKFNaive / kfIKFFull covariance update uses the numerically stable
Joseph form `P' = (I-KH)*P*(I-KH)^T + K*R*K^T` instead of the classic
`P' = (I-KH)*P`. Set to `false` for slightly faster (but less robust) updates.



---

## 21. `COccupancyGridMap2D` API modernization

Several breaking changes were made to `mrpt::maps::COccupancyGridMap2D` for
MRPT 3.x. Update callers as follows:

### Voronoi spelling fixes
- `getVoroniClearance()` → `getVoronoiClearance()`
- `setVoroniClearance()` → `setVoronoiClearance()` (protected)
- member `voroni_free_threshold` → `m_voronoiFreeThreshold` (protected)

### Cell-size compile-time switch removed
The `OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS` / `_16BITS` macros and the CMake
option `MRPT_OCCUPANCY_GRID_CELLSIZE` have been removed. `cellType` is always
`int8_t`. Old 8-bit and 16-bit serialized streams (v0–v6) remain readable.

### `computeClearance` now returns a struct
```cpp
// Before:
int basis_x[2], basis_y[2], nBasis;
int clearance = grid.computeClearance(cx, cy, basis_x, basis_y, &nBasis);

// After:
auto res = grid.computeClearance(cx, cy);
// res.clearance, res.nBasis, res.basisX[0/1], res.basisY[0/1]
```

### `computeLikelihoodField_Thrun` / `_II` signatures changed
```cpp
// Before:
double lk = grid.computeLikelihoodField_Thrun(pPointsMap, &pose);
double lk = grid.computeLikelihoodField_Thrun(pPointsMap, nullptr);

// After:
double lk = grid.computeLikelihoodField_Thrun(*pPointsMap, pose);
double lk = grid.computeLikelihoodField_Thrun(*pPointsMap);  // no pose
```

### `saveAsBitmapTwoMapsWithCorrespondences` takes const references
```cpp
// Before:
COccupancyGridMap2D::saveAsBitmapTwoMapsWithCorrespondences(file, &m1, &m2, corrs);

// After:
COccupancyGridMap2D::saveAsBitmapTwoMapsWithCorrespondences(file, m1, m2, corrs);
```

### Ray-trace step size moved from global static to per-instance option
```cpp
// Before:
COccupancyGridMap2D::RAYTRACE_STEP_SIZE_IN_CELL_UNITS = 0.5;

// After:
grid.insertionOptions.raytraceStepSizeInCellUnits = 0.5;
```

### Critical points: struct-of-arrays replaced by array-of-structs
```cpp
// Before (public member):
grid.CriticalPointsList.x[i], .y[i], .clearance[i]

// After (const accessor):
for (const auto& cp : grid.criticalPoints()) {
    // cp.x, cp.y, cp.clearance
}
```

### `getAsImage` now takes an options struct
```cpp
// Before:
grid.getAsImage(img, /*verticalFlip=*/false, /*forceRGB=*/false, /*tricolor=*/true);

// After:
COccupancyGridMap2D::TGetAsImageParams p;
p.tricolor = true;
grid.getAsImage(img, p);

// Zero-argument form still works:
grid.getAsImage(img);
```

### Internal members `updateInfoChangeOnly` and `likelihoodOutputs` privatized
These were public mutable members used as hidden in/out channels. They have
been renamed (`m_updateInfoChangeOnly`, `m_likelihoodOutputs`) and moved to
the `protected` section. External code that directly accessed them must be
refactored to use `computeObservationLikelihood()` instead.
