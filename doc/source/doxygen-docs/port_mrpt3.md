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

## 12. Step-by-step migration checklist

1. **CMakeLists.txt**: Replace `mrpt-foo` → `mrpt_foo` in `find_package` calls,
   and `mrpt::foo` → `mrpt::mrpt_foo` in `target_link_libraries`.
   Add `find_package(mrpt_common REQUIRED)` and use `mrpt_add_executable()`.

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

6. **`loadFromFile`**: Replace integer channel args with `TImageChannels` enum.

7. **`CFBORender`**: Replace `getCamera(scene)` with `setCamera()` /
   `getCameraOverride()`.

8. **Remove calls** to deleted `global_settings::OCTREE_*`, octree query
   methods, `matchFeatures`, and other removed APIs.

9. **`prepareVideoSourceFromUserSelection`**: Move from `mrpt::hwdrivers` to
   `mrpt::apps`, add `mrpt_libapps_gui` dependency.

10. **Build and iterate**: Use `colcon build --packages-up-to mrpt_<module>`
    to build incrementally and fix remaining issues.

---

## 13. Known pending work / TO-DO list for MRPT 3.0

This section tracks items that are present in the codebase as stubs, throw
run-time `THROW_EXCEPTION("TODO")`, carry `MRPT_TODO` or `// TODO` markers, or
are otherwise visibly incomplete after the 2.x → 3.0 porting effort.

### 13.1 `mrpt_viz` — Missing rendering implementations

Several scene-graph classes were ported (serialization, bounding box, setters)
but **never received an `updateBuffers()` override**, which means they compile
but produce **invisible objects** at run-time.  The new rendering pipeline
(mrpt_opengl proxy objects) requires each concrete `CVisualObject` subclass
to override `CVisualObject::updateBuffers()` to fill the CPU-side
`VisualObjectParams_*` buffers that are subsequently uploaded to the GPU.

| Class | Issue |
|-------|-------|
| `mrpt::viz::CColorBar` | ~~No `updateBuffers()`~~ **DONE** (triangles for the gradient bar, lines for tick marks) |
| `mrpt::viz::CGridPlaneXZ` | ~~No `updateBuffers()`~~ **DONE** |
| `mrpt::viz::CVectorField2D` | ~~No `updateBuffers()`~~ **DONE** (lines + arrowhead triangles + points) |
| `mrpt::viz::CVectorField3D` | ~~No `updateBuffers()`~~ **DONE** (lines + points, with optional module-based colormap) |
| `mrpt::viz::CMesh3D` | ~~No `updateBuffers()`~~ **DONE** (triangles for faces, lines for edges) |
| `mrpt::viz::COctoMapVoxels` | ~~No `updateBuffers()`~~ **DONE** (triangles for solid voxels, lines for grid, points mode) |
| `mrpt::viz::CText` | `computeTextExtension()` body wrapped in `#if 0` and throws `THROW_EXCEPTION("TODO")` with `MRPT_TODO("Refactor!")` — needs reimplementation with the nanogui/gltext backend |
| `mrpt::viz::CText3D` | Similar to `CText` — 3-D text rendering missing |
| `mrpt::viz::CPlanarLaserScan` | ~~No `updateBuffers()`~~ **DONE** (lines for scan outline, triangles for surface, points for endpoints) |
| `mrpt::viz::CFrustum` | `traceRay()` throws `THROW_EXCEPTION("TO DO")` |
| `mrpt::viz::CSetOfTexturedTriangles` | `traceRay()` throws `std::runtime_error("TODO: TraceRay not implemented")` |

Additional partial issues in `mrpt_viz`:

- **`CPolyhedron`**: The code has some TO-DOs inherited from 2.x. Won't implement for 3.0.0.
- **`CAssimpModel`**: Compressed embedded textures are silently skipped
  (`"Compressed embedded textures not yet supported"`, `CAssimpModel.cpp:715`).
  Transparent/textured meshes lack proper spatial subdivision for correct
  transparency sorting (`// TODO: Implement spatial subdivision…`, line 861;
  current code is a `// For now, this is a no-op placeholder`).
- **`CPointCloud` / `CPointCloudColoured`**: Efficiency notes
  (`// JL: TODO note: Well, this can be clearly done much more efficiently`)
  for the inner rendering loops (lines 214, 231 and 87, 102 respectively).

### 13.2 `mrpt_opengl` — Rendering pipeline gaps

- **Frustum culling not implemented**: `CompiledViewport.cpp:984` has a
  `// TODO: Implement frustum culling using proxy->getBoundingBox()` comment;
  every object is currently rendered unconditionally.
- **16-bit depth texture**: `Texture.cpp:334` throws
  `THROW_EXCEPTION("todo: textures with D16U depth")`.
- **Shadow rendering**: The latest commit (`afc33a207`) marks shadow rendering
  as an "attempt at fix"; the shadow map pipeline (directional light pass) may
  still have correctness issues worth re-verifying.

### 13.3 `mrpt_img` — Image processing stubs (post-OpenCV removal)

Several methods that previously delegated to OpenCV were re-declared but not
reimplemented with the STB/custom backend:

| Method | Status |
|--------|--------|
| `CImage::normalize()` | ~~Throws~~ **DONE** — finds min/max and rescales all pixels to [0,255] |
| `CImage::undistort()` | **Implemented** — uses `CUndistortMap` internally; supports all distortion models |
| `CImage::filterMedian()` | ~~Body wrapped in `#if 0`~~ **DONE** — manual median filter with border clamping |
| `CImage::filterGaussian()` | ~~Body wrapped in `#if 0`~~ **DONE** — separable Gaussian kernel with bilinear border clamping |
| `CImage::rotateImage()` | ~~Throws~~ **DONE** — inverse-mapping bilinear interpolation with zero-border fill |
| `CImage::drawChessboardCorners()` | ~~Throws~~ **DONE** — draws cross+circle per corner and connecting lines per row using `line()`/`drawCircle()` |
| `CImage::drawImage()` (cross-channel) | ~~Throws~~ **DONE** — handles grayscale↔RGB/RGBA conversion (luminance formula and gray replication) |

### 13.4 `mrpt_poses` — Unimplemented PDF operations

Many probability distribution classes have methods that were declared but never
implemented, throwing `THROW_EXCEPTION("TODO!!!")` or
`THROW_EXCEPTION("Not implemented yet!")` at run-time:

- **`CPosePDFGrid`**: `normalizeWeights()`, `getMostLikelyCPose()`,
  `getCovarianceAndMean()`, `drawSingleSample()`,
  `bayesianFusion()`, `inverse()` — all throw.
- **`CPose3DPDFGrid`**: `getMostLikelyCPose()`, `getCovarianceAndMean()`,
  `drawSingleSample()`, `bayesianFusion()`, `inverse()` and others — all throw.
- **`CPose3DPDFParticles`**: `getCovarianceAndMean()` (`"TO DO!!"`),
  `bayesianFusion()`, `evaluateNormalizedPDF()`,
  `evaluatePDF()`, `changeCoordinatesReference()` — all throw.
- **`CPose3DPDFSOG`**: `bayesianFusion()` (`"TODO!!!"`),
  `evaluateNormalizedPDF()`, `drawSingleSample()` — all throw.
- **`CPose3DPDFGaussian`**: `bayesianFusion()`, `inverse()`,
  `drawSingleSample()` — all throw `"TO DO!!!"`.
- **`CPose3DPDFGaussianInf`**: `bayesianFusion()`, `inverse()`,
  `drawSingleSample()` — all throw `"TO DO!!!"`.
- **`CPointPDFGaussian::drawSingleSample()`** — throws `"TODO!!!"`.
- **`CPoint2DPDFGaussian::drawSingleSample()`** — throws `"TODO!!!"`.
- **`CPointPDFParticles`**: `evaluateNormalizedPDF()`, `drawSingleSample()` — throw.
- **`CPosePDFParticles::evaluateNormalizedPDF()`** — throws `"Not implemented yet!"`.
- **`CPoseRandomSampler`**: sampling for non-Gaussian 2-D and 3-D PDFs throws
  `THROW_EXCEPTION("TODO")` (lines 260, 308 of `CPoseRandomSampler.cpp`).

### 13.5 `mrpt_slam` — Incomplete algorithms

- **`CICP` (3-D mode)**: Only `icpClassic` is implemented for 3-D ICP;
  other variants throw `THROW_EXCEPTION("Only icpClassic is implemented for ICP-3D")`.
- **`path_from_rtk_gps`**: GPS path reconstruction is hardcoded for exactly
  3 receivers (`// TODO: Generalize equations for # of GPS > 3`, lines 175, 213).
- **`data_association`**: Joint compatibility branch-and-bound has a
  `// TODO: Optimized version!!` note (line 583).
- **`CRejectionSamplingRangeOnlyLocalization`**: Two open notes about
  sensor-height handling and sigma being taken from a fixed field rather than
  the observation's `stdError`.

### 13.6 `mrpt_nav` — Planner optimisation

- **`PlannerRRT_SE2_TPS::getNearestNode()`**: Marked with
  `MRPT_TODO("Optimize getNearestNode() with KD-tree!")` — currently does a
  linear scan over all nodes.
- **`CParameterizedTrajectoryGenerator`**: One obstacle post-processing enum
  value is not handled and throws.

### 13.7 `mrpt_maps` — Miscellaneous

- **`CAngularObservationMesh`**: Line 134 carries `// TODO: redo` with no
  further explanation — the rendering/update path likely needs review.
- **`CObservationRotatingScan`**: `// TODO: populate organizedPoints?`
  (line 362) — the organized point-cloud representation is never filled.
- **`COccupancyGridMap2D` (multi-pose insertion)**: `FIXME: doesn't support
  many different poses in one measurement` in `COccupancyGridMap2D_insert.cpp`.
- **`CGasConcentrationGridMap2D` / `CWirelessPowerGridMap2D`**: Serialization
  loops carry `// TODO: Do this endianness safe!!` notes.

### 13.8 `mrpt_math`

- **`KDTreeCapable` unit test**: The only test in
  `KDTreeCapable_unittest.cpp` consists of a single `MRPT_TODO("Write me!")`
  call — there are no actual test cases.

### 13.9 `mrpt_graphs`

- **`ScalarFactorGraph`**: `// MRPT_TODO("Use compressed access instead of
  coeff() below")` — sparse-matrix access uses the slow `coeff()` path.

### 13.10 Documentation pages needing v3 updates

Several Doxygen documentation pages still reference MRPT 2.x APIs or are
minimal stubs:

| Page | Issue |
|------|-------|
| `lib_mrpt_slam.md` | References `mrpt-slam` (hyphen) in text; could expand on available algorithms |
| `lib_mrpt_nav.md` | Stub — minimal description of reactive navigation |
| `lib_mrpt_hwdrivers.md` | Stub — no list of supported sensor classes |
| `lib_mrpt_core.md` | Stub — needs description of core utilities |
| `lib_mrpt_poses.md` | Stub — needs overview of pose PDF classes |
| `lib_mrpt_obs.md` | Needs review for v3 API changes |
| `lib_mrpt_math.md` | Needs review — should document Eigen integration |

### 13.11 Python bindings (`pybind11`)

Several modules are documented as "not yet implemented":

- `mrpt_nav` (Phase 1.6): `CReactiveNavigationSystem`, waypoints API —
  complex due to virtual callbacks.
- `mrpt_bayes` (Phase 2.2): template-heavy Kalman/particle filter classes.
- `mrpt_topography` (Phase 2.5): free functions for geographic conversions.
- `mrpt_vision` (Phase 2.6): feature detection/matching (module being
  deprecated; target package is `mola_vision`).
- `mrpt_hwdrivers`, `mrpt_comms`, `mrpt_graphslam`, `mrpt_libapps_cli`
  (Phases 2.7–2.10): low-priority.
- Python examples (`mrpt_examples_py`): incomplete — examples are needed
  for each newly wrapped module.
