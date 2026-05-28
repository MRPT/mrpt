\page porting_mrpt3 Porting code from MRPT 2.x to MRPT 3.0

# Porting guide: MRPT 2.x → MRPT 3.0

This guide documents every breaking change in MRPT 3.0 and shows how to migrate
code written against MRPT 2.x.

It is organized so that you read it top-to-bottom once, then use it as a
reference:

- **[Why MRPT 3.0?](#why)** — rationale for the refactoring.
- **[Migration checklist](#checklist)** — the fast path; start here.
- **[Build system](#build)** — CMake packages, targets, and removed headers.
- **[Namespaces & renames](#namespaces)** — `mrpt::opengl` → `mrpt::viz`.
- **[Cross-cutting API patterns](#patterns)** — mechanical changes that recur
  across many classes (return-by-value, `std::optional`, scoped enums,
  coordinate/option aggregates). **Most edits fall under one of these four.**
- **[Module-by-module changes](#modules)** — specifics that are not covered by a
  generic pattern.
- **[Removed APIs](#removed)** — a single lookup table.
- **[Porting ROS 2 nodes](#ros)** — for downstream packages that used `mrpt_ros`.

> **Conventions used below.** Code blocks are always labeled `// MRPT 2.x:`
> (old) and `// MRPT 3.0:` (new). APIs marked *(deprecated)* still compile with
> a warning; APIs marked *(removed)* do not exist anymore and must be changed.

> **Note for Python users.** Pybind11 binding conventions live in the
> repository `AGENTS.md`, not here. This guide covers the C++ API only.

---

<a name="why"></a>
## Why MRPT 3.0? — Advantages of the refactoring

- **Modular colcon-based build.** Each MRPT module (`mrpt_core`, `mrpt_math`,
  `mrpt_poses`, `mrpt_viz`, …) is an independent colcon package with its own
  `CMakeLists.txt`. No ROS dependency is required — plain colcon is the
  meta-build tool — enabling faster incremental builds and independent
  versioning.
- **Clean namespace separation.** The scene-graph / 3-D visualization API now
  lives in `mrpt::viz`, while `mrpt::opengl` is reserved for low-level GPU
  rendering internals (shaders, FBOs, textures). This shrinks the public API
  surface and makes it easier to learn.
- **Modern C++ API.** Output-reference parameters replaced by `std::optional`
  returns or return-by-value, raw coordinate pairs replaced by `TPixelCoord`,
  integer channel arguments replaced by the `TImageChannels` enum, and scoped
  `enum class` types throughout.
- **Less legacy code.** Octree rendering settings, sparse matrices, old feature
  matching, `CColouredPointsMap`, and other rarely-used APIs were removed,
  reducing maintenance burden and binary size.

---

<a name="checklist"></a>
## Migration checklist

A typical port touches these steps in order. Each links to its detailed
section.

1. **CMakeLists.txt** — rename `mrpt-foo` → `mrpt_foo` in `find_package`, and
   `mrpt::foo` → `mrpt::mrpt_foo` in `target_link_libraries`. Optionally adopt
   `mrpt_add_executable()`. See [Build system](#build).
2. **Headers** — replace `#include <mrpt/opengl/Foo.h>` with
   `#include <mrpt/viz/Foo.h>` for scene-graph classes; keep `mrpt/opengl/`
   only for the low-level rendering classes. See [Namespaces](#namespaces).
3. **Namespaces** — replace `mrpt::opengl::` → `mrpt::viz::` and `opengl::` →
   `viz::` (including `using namespace`). See [Namespaces](#namespaces).
4. **Class names** — `COpenGLScene` → `Scene`, `COpenGLViewport` → `Viewport`,
   `CRenderizable` → `CVisualObject`. See [Namespaces](#namespaces).
5. **Drawing calls** — convert `textOut(x, y, …)`, `line(x0, y0, x1, y1, …)`,
   `filledRectangle(…)` to `TPixelCoord` / brace-init. See [§4 Coordinate &
   option aggregates](#aggregates).
6. **Output-reference calls** — replace `foo(out)` with `out = foo()` for the
   modernized methods. See [§1 Return-by-value](#by-value).
7. **`bool`-returning queries** — adopt the `std::optional` versions. See
   [§2 std::optional](#optional).
8. **Enum arguments** — replace integer/`bool` flags with the new scoped enums
   and option structs. See [§3 Scoped enums](#enums).
9. **Removed APIs** — delete or replace calls listed in [Removed APIs](#removed).
10. **Build & iterate** — `colcon build --packages-up-to mrpt_<module>`, then
    fix remaining errors.

---

<a name="build"></a>
## Build system

### Package and target renaming

The two mechanical rules cover almost all CMake edits:

| MRPT 2.x | MRPT 3.0 |
|---|---|
| `find_package(mrpt-<X>)` | `find_package(mrpt_<X>)` |
| `mrpt::<X>` (target) | `mrpt::mrpt_<X>` |

Examples: `find_package(mrpt-poses)` → `find_package(mrpt_poses)`;
`mrpt::gui` → `mrpt::mrpt_gui`; `mrpt::slam` → `mrpt::mrpt_slam`.

### Example `CMakeLists.txt`

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

### Removed headers

| Removed header | Replacement |
|---|---|
| `<mrpt/opengl.h>` (catch-all) | Individual `<mrpt/viz/...>` headers |
| `<mrpt/examples_config.h>` (`MRPT_EXAMPLES_BASE_DIRECTORY`) | CMake compile definition (below) |
| `<mrpt/config.h>` | Module-specific config headers (e.g. from `mrpt_opengl`) |

For the examples base directory, define it yourself:

```cmake
add_compile_definitions(
    MRPT_EXAMPLES_BASE_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/")
```

---

<a name="namespaces"></a>
## Namespaces, headers & class renames

### `mrpt::opengl` → `mrpt::viz`

The entire scene-graph API (3-D objects, scenes, viewports, cameras, stock
objects) moved from `mrpt::opengl` to `mrpt::viz`:

```cpp
// MRPT 2.x:
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/Scene.h>
mrpt::opengl::CPointCloud pc;
opengl::stock_objects::CornerXYZ();

// MRPT 3.0:
#include <mrpt/viz/CPointCloud.h>
#include <mrpt/viz/Scene.h>
mrpt::viz::CPointCloud pc;
viz::stock_objects::CornerXYZ();
```

### What stays in `mrpt::opengl`

Only low-level GPU rendering internals:

| Class / header | Header |
|---|---|
| `mrpt::opengl::CFBORender` | `<mrpt/opengl/CFBORender.h>` |
| `mrpt::opengl::Shader` | `<mrpt/opengl/Shader.h>` |
| `mrpt::opengl::Buffer` | `<mrpt/opengl/Buffer.h>` |
| `mrpt::opengl::Texture` | `<mrpt/opengl/Texture.h>` |
| `mrpt::opengl::DefaultShaders` | `<mrpt/opengl/DefaultShaders.h>` |
| (low-level GL API) | `<mrpt/opengl/opengl_api.h>` |

### Class renames

| MRPT 2.x | MRPT 3.0 |
|---|---|
| `COpenGLScene` | `mrpt::viz::Scene` |
| `COpenGLViewport` | `mrpt::viz::Viewport` |
| `CRenderizable` | `mrpt::viz::CVisualObject` |

---

<a name="patterns"></a>
## Cross-cutting API patterns

Four mechanical patterns account for most non-rename edits. Learn each rule
once, then apply it to every affected API listed.

<a name="by-value"></a>
### 1. Output-reference parameters → return-by-value

Methods that wrote a result into an output reference now **return it**. The
out-parameter overloads are kept as *deprecated* wrappers where noted.

**Rule:** `obj.foo(out);` → `auto out = obj.foo();`

```cpp
// MRPT 2.x (deprecated):
mrpt::poses::CPose3D pose;
obs->getSensorPose(pose);
mrpt::math::CMatrixDouble44 HM;
pose.getHomogeneousMatrix(HM);

// MRPT 3.0:
auto pose = obs->getSensorPose();
auto HM   = pose.getHomogeneousMatrix();
```

Affected APIs:

| API | MRPT 3.0 form |
|---|---|
| `CObservation::getSensorPose(CPose3D&)` | `CPose3D getSensorPose()` — now `virtual`, overridden by all concrete observations |
| `CPose3D/CPose2D/CPose3DQuat::getHomogeneousMatrix(M)` | returns the matrix |
| `CPose3D::getRotationMatrix(M)` | returns the matrix |
| `CPoseRandomSampler::getOriginalPDFCov2D(cov)` | `cov = getOriginalPDFCov2D()` (also `getOriginalPDFCov3D()`) |
| `CPose3DPDFGaussian::getCovSubmatrix2D(out)` | returns the submatrix |
| `CAngularObservationMesh::generateSetOfTriangles(tris)` | returns the triangles |
| `CAngularObservationMesh::getTracedRays(rays)` | returns the rays |
| `CAngularObservationMesh::getUntracedRays(rays, dist)` | `getUntracedRays(dist)` |
| `CGasConcentrationGridMap2D::getWindAs3DObject(obj)` | returns the object |
| `circular_buffer::pop(val)` | `val = pop()` |
| `CRejectionSamplingCapable::rejectionSampling(N, out, M)` | `out = rejectionSampling(N, M)` |

<a name="optional"></a>
### 2. `bool` + out-param → `std::optional`

Queries that returned `bool` and wrote to an out-reference now return
`std::optional<T>`.

**Rule:** `if (obj.foo(out)) {…}` → `if (auto r = obj.foo()) { /* use *r */ }`

```cpp
// MRPT 2.x:
mrpt::math::TLine3D ray;
bool ok = vp->get3DRayForPixelCoord(x, y, ray);

// MRPT 3.0:
auto ray = vp->get3DRayForPixelCoord({x, y});
if (ray.has_value()) { /* use *ray */ }
```

Affected APIs:

- `CDisplayWindow3D` / viewport `get3DRayForPixelCoord(x, y, ray)` →
  `get3DRayForPixelCoord({x, y})` (note: also adopts `TPixelCoord`, see
  [§4](#aggregates)).
- Hardware grabbers: `getObservation(obs)` *(deprecated)* → `grabFrame()`
  returning `std::optional<T>`. Affects `CImageGrabber_OpenCV`,
  `CImageGrabber_dc1394`, `CMyntEyeCamera`, `CFFMPEG_InputStream` (→
  `std::optional<CImage>`), `CEnoseModular`, `CWirelessPower`.

```cpp
// MRPT 2.x (deprecated):
mrpt::obs::CObservationImage obs;
bool ok = camera.getObservation(obs);

// MRPT 3.0:
auto obs_opt = camera.grabFrame();
if (obs_opt) { /* use *obs_opt */ }
```

<a name="enums"></a>
### 3. Scoped `enum class` replacements

Integer / `bool` flag arguments became scoped enums. Old enumerator names that
still compile are noted.

```cpp
// MRPT 2.x:
iface.stop(true);                       // isEmergencyStop
map.savePCDFile("out.pcd", true);       // binary
if (e == CRawlog::etSensoryFrame) {}

// MRPT 3.0:
iface.stop(mrpt::nav::StopType::Emergency);   // or ::Normal
map.savePCDFile("out.pcd", mrpt::maps::PCDFormat::Binary);   // or ::ASCII
if (e == CRawlog::TEntryType::etSensoryFrame) {}
```

`CParticleFilter` algorithm/resampling enums are now `enum class` with
PascalCase names; the old `pf*` / `pr*` names remain as `static constexpr`
aliases but the new names are preferred:

| Old name (still valid) | New name |
|---|---|
| `CParticleFilter::pfStandardProposal` | `TParticleFilterAlgorithm::StandardProposal` |
| `CParticleFilter::pfAuxiliaryPFStandard` | `TParticleFilterAlgorithm::AuxiliaryPFStandard` |
| `CParticleFilter::pfOptimalProposal` | `TParticleFilterAlgorithm::OptimalProposal` |
| `CParticleFilter::pfAuxiliaryPFOptimal` | `TParticleFilterAlgorithm::AuxiliaryPFOptimal` |
| `CParticleFilter::prMultinomial` | `TParticleResamplingAlgorithm::Multinomial` |
| `CParticleFilter::prResidual` | `TParticleResamplingAlgorithm::Residual` |
| `CParticleFilter::prStratified` | `TParticleResamplingAlgorithm::Stratified` |
| `CParticleFilter::prSystematic` | `TParticleResamplingAlgorithm::Systematic` |

(New names are nested under `CParticleFilter::`.)

<a name="aggregates"></a>
### 4. Coordinate & option aggregates

Multiple scalar arguments were grouped into small structs (brace-init friendly)
or option structs.

**Pixel coordinates** — `CCanvas` / `CImage` drawing methods take `TPixelCoord`:

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

**3-D points** — `CArrow::Create` takes `TPoint3Df` endpoints:

```cpp
// MRPT 2.x:
auto a = CArrow::Create(x0, y0, z0, x1, y1, z1, headRatio, smallR, largeR);
// MRPT 3.0:
auto a = CArrow::Create({x0, y0, z0}, {x1, y1, z1}, headRatio, smallR, largeR);
```

**Option structs** — boolean argument lists became named-field structs:

```cpp
// MRPT 2.x:
mrpt::math::vectorToTextFile(v, "file.txt", false, true);  // append, byRows
// MRPT 3.0:
mrpt::math::VectorTextFileOptions opts;
opts.append = false;
opts.byRows = true;
mrpt::math::vectorToTextFile(v, "file.txt", opts);
```

See also `COccupancyGridMap2D::getAsImage` (`TGetAsImageParams`) and
`computeClearance` (struct return) in [§ mrpt_maps](#maps).

---

<a name="modules"></a>
## Module-by-module changes

Changes below are not covered by a generic pattern and need per-API attention.

### mrpt_img

**`CImage::loadFromFile` channel argument** — `int` → `TImageChannels` enum:

```cpp
// MRPT 2.x:
img.loadFromFile("file.png", 0);   // 0 = grayscale
// MRPT 3.0:
img.loadFromFile("file.png", mrpt::img::CH_GRAY);
// Other values: CH_RGB, CH_RGBA, CH_AS_IS
```

**`CImage::operator()` removed** — use `ptr<T>()`:

```cpp
// MRPT 2.x:
*img(x, y) = 128;
// MRPT 3.0:
*img.ptr<uint8_t>(x, y) = 128;
```

**`CVideoFileWriter` moved** from `mrpt::vision` to `mrpt::img`
(`<mrpt/img/CVideoFileWriter.h>`). It no longer supports MP4 — only simpler AVI
formats — but is self-contained without external libraries.

### mrpt_viz / mrpt_opengl

**`TLightParameters` multi-light API** — the single directional light
(flat `direction`/`diffuse`/`specular`/`color` fields) is replaced by a
`std::vector<TLight> lights` supporting up to 8 simultaneous lights of type
`TLightType::Directional`, `Point`, or `Spot`:

```cpp
// MRPT 2.x:
auto& lp = viewport->lightParameters();
lp.direction = {-0.4f, -0.4f, -0.8f};
lp.diffuse   = 0.8f;
lp.specular  = 0.95f;
lp.color     = {1.0f, 1.0f, 1.0f};

// MRPT 3.0:
auto& lp = viewport->lightParameters();
lp.lights[0].direction = {-0.4f, -0.4f, -0.8f};   // the default directional light
lp.lights[0].diffuse   = 0.8f;
lp.lights[0].specular  = 0.95f;
lp.lights[0].color     = {1.0f, 1.0f, 1.0f};

// Add a point light:
lp.lights.push_back(mrpt::viz::TLight::PointLight(
    {5.0f, 0.0f, 3.0f},   // position
    {1.0f, 0.9f, 0.8f})); // color
```

The global `ambient` field stays on `TLightParameters`. Shadow mapping still
applies only to the first directional light.

**`CFBORender` camera** — `getCamera(scene)` *(removed)* is replaced by
`setCamera()` / `getCameraOverride()`:

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

`CArrow` constructor changes are under [§4 aggregates](#aggregates).

<a name="maps"></a>
### mrpt_maps — `COccupancyGridMap2D`

**Voronoi spelling fixes:**
- `getVoroniClearance()` → `getVoronoiClearance()`
- `setVoroniClearance()` → `setVoronoiClearance()` (now `protected`)
- member `voroni_free_threshold` → `m_voronoiFreeThreshold` (`protected`)

**Cell-size compile-time switch removed.** The
`OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS` / `_16BITS` macros and the CMake option
`MRPT_OCCUPANCY_GRID_CELLSIZE` are gone; `cellType` is always `int8_t`. Old
8-bit and 16-bit serialized streams (v0–v6) remain readable.

**`computeClearance` returns a struct:**

```cpp
// MRPT 2.x:
int basis_x[2], basis_y[2], nBasis;
int clearance = grid.computeClearance(cx, cy, basis_x, basis_y, &nBasis);
// MRPT 3.0:
auto res = grid.computeClearance(cx, cy);
// res.clearance, res.nBasis, res.basisX[0/1], res.basisY[0/1]
```

**`computeLikelihoodField_Thrun` / `_II` signatures** take a reference and an
optional pose:

```cpp
// MRPT 2.x:
double lk = grid.computeLikelihoodField_Thrun(pPointsMap, &pose);
double lk = grid.computeLikelihoodField_Thrun(pPointsMap, nullptr);
// MRPT 3.0:
double lk = grid.computeLikelihoodField_Thrun(*pPointsMap, pose);
double lk = grid.computeLikelihoodField_Thrun(*pPointsMap);   // no pose
```

**`saveAsBitmapTwoMapsWithCorrespondences`** takes const references, not
pointers:

```cpp
// MRPT 2.x:
COccupancyGridMap2D::saveAsBitmapTwoMapsWithCorrespondences(file, &m1, &m2, corrs);
// MRPT 3.0:
COccupancyGridMap2D::saveAsBitmapTwoMapsWithCorrespondences(file, m1, m2, corrs);
```

**Ray-trace step size** moved from a global static to a per-instance option:

```cpp
// MRPT 2.x:
COccupancyGridMap2D::RAYTRACE_STEP_SIZE_IN_CELL_UNITS = 0.5;
// MRPT 3.0:
grid.insertionOptions.raytraceStepSizeInCellUnits = 0.5;
```

**Critical points** — struct-of-arrays → array-of-structs (const accessor):

```cpp
// MRPT 2.x (public member):
grid.CriticalPointsList.x[i], .y[i], .clearance[i]
// MRPT 3.0 (const accessor):
for (const auto& cp : grid.criticalPoints()) {
    // cp.x, cp.y, cp.clearance
}
```

**`getAsImage`** takes an options struct:

```cpp
// MRPT 2.x:
grid.getAsImage(img, /*verticalFlip=*/false, /*forceRGB=*/false, /*tricolor=*/true);
// MRPT 3.0:
COccupancyGridMap2D::TGetAsImageParams p;
p.tricolor = true;
grid.getAsImage(img, p);
grid.getAsImage(img);   // zero-argument form still works
```

**Privatized members.** `updateInfoChangeOnly` and `likelihoodOutputs` (public
mutable in/out channels in 2.x) are renamed `m_updateInfoChangeOnly` /
`m_likelihoodOutputs` and moved to `protected`. External code that read them
must use `computeObservationLikelihood()` instead.

### mrpt_bayes

**`CKalmanFilterCapable::OnSubstractObservationVectors` renamed** (typo fix) to
`OnSubtractObservationVectors`. The old name is a deprecated trampoline:

```cpp
// MRPT 2.x (compiles with deprecation warning):
void MyKF::OnSubstractObservationVectors(KFArray_OBS& A, const KFArray_OBS& B) const override;
// MRPT 3.0:
void MyKF::OnSubtractObservationVectors(KFArray_OBS& A, const KFArray_OBS& B) const override;
```

**`TKF_options::use_joseph_form` (new option, default `true`)** controls whether
the `kfEKFNaive` / `kfIKFFull` covariance update uses the numerically stable
Joseph form `P' = (I-KH)·P·(I-KH)ᵀ + K·R·Kᵀ` instead of the classic
`P' = (I-KH)·P`. Set to `false` for slightly faster but less robust updates.

**`CParticleFilterCapable::TParticleProbabilityEvaluator` is now
`std::function`.** Existing plain or static-member function pointers still work
unchanged; lambdas and functors can now be passed directly without wrapping.

Particle-filter enum migration is under [§3 scoped enums](#enums); rejection
sampling is under [§1 return-by-value](#by-value).

### mrpt_hwdrivers

**`prepareVideoSourceFromUserSelection`** moved from `mrpt::hwdrivers` to
`mrpt::apps` (header `<mrpt/apps_gui/CameraSelectionGUI.h>`). Add
`mrpt_libapps_gui` to your CMake dependencies.

Grabber `grabFrame()` changes are under [§2 std::optional](#optional).

### mrpt_containers — `yaml`

**`map_t` is now an insertion-order vector of pairs.** In 2.x `yaml::map_t` was
`std::map<std::string, node_t>`; in 3.x it is
`std::vector<std::pair<node_t, node_t>>`. **Do not call `std::map` methods on the
raw `map_t` returned by `asMap()`** — use the high-level `yaml` API:

```cpp
// MRPT 2.x:
const auto& m = node.asMap();
if (m.count("key")) { ... }
auto val = m.at("key").as<double>();
// MRPT 3.0:
if (node.has("key")) { ... }
auto val = node["key"].as<double>();
```

When iterating a sequence and needing subscript access per element, wrap the
`node_t` in a temporary `yaml`:

```cpp
// MRPT 2.x:
for (auto& it : seq.asSequence()) {
  auto& m = it.asMap();
  auto v = m.at("field").as<double>();
}
// MRPT 3.0:
for (const auto& it : seq.asSequence()) {
  const mrpt::containers::yaml item(it);
  auto v = item["field"].as<double>();
}
```

**Assigning into a `sequence_t` slot.** `sequence_t` is `std::vector<node_t>`,
so extract the underlying `node_t` with `.node()`:

```cpp
// MRPT 2.x:
seq.asSequence().at(i) = std::move(myYaml);
// MRPT 3.0:
seq.asSequence().at(i) = std::move(myYaml.node());
```

To append, use `yaml::push_back(const yaml&)` on the owning `yaml`:

```cpp
// MRPT 2.x:
d.asSequence().push_back(entry);   // ERROR: not defined on vector<node_t>
// MRPT 3.0:
d.push_back(entry);                // uses yaml::push_back(const yaml&)
```

**`yaml_ref` / `yaml_cref` proxy types.** `operator[]` on a non-`const yaml`
returns a mutable `yaml_ref`; on a `const yaml` (or from subscripting a
`yaml_ref`) it returns a read-only `yaml_cref`. Both forward `as<T>()`,
`has()`, `size()`, `isMap()`, `isSequence()`, `isScalar()`, `toMatrix()`,
`toStdVector<>()`, and `printAsYAML()`, and implicitly convert to `yaml` (deep
copy) when needed. The proxies do **not** expose `internalPushBack` or raw
`asMap()`/`asSequence()` mutation — call those on the owning `yaml` object or
obtain a `yaml` copy first.

---

<a name="removed"></a>
## Removed APIs

No direct replacement unless noted.

| Removed API | Notes / replacement |
|---|---|
| `mrpt::vision` | Removed (a few classes moved to `mrpt::img`); CV features will move to a new `mola_vision` package |
| `mrpt::math::CSparseMatrix` | Use Eigen sparse matrices |
| `mrpt::maps::CColouredPointsMap` | Use `CSimplePointsMap` or `CPointsMapXYZI` |
| `CLandmarksMap::insertionOptions` / `likelihoodOptions` | Removed |
| `global_settings::OCTREE_RENDER_MAX_DENSITY_POINTS_PER_SQPIXEL` | Octree rendering removed |
| `global_settings::OCTREE_RENDER_MAX_POINTS_PER_NODE` | Octree rendering removed |
| `CPointCloud::octree_get_graphics_boundingboxes()` | Octree API removed |
| `CPointCloud::getActuallyRendered()` | Octree API removed |
| `CPointCloud::octree_get_visible_nodes()` | Octree API removed |
| `CPointCloud::octree_get_node_count()` | Octree API removed |
| `#include <mrpt/opengl.h>` | Use individual `<mrpt/viz/...>` headers |
| `#include <mrpt/examples_config.h>` | Use a CMake compile definition |
| `#include <mrpt/config.h>` | Use module-specific config headers |

---

<a name="ros"></a>
## Porting ROS 2 nodes from `mrpt_ros` (2.x) to native MRPT 3.x

Starting with MRPT 3.0, the upstream [MRPT/mrpt](https://github.com/MRPT/mrpt)
repository is itself a colcon-friendly multi-package workspace, so the
`mrpt_ros` wrapper is no longer needed. Downstream ROS 2 packages should depend
directly on the native `mrpt_<module>` packages shipped by MRPT 3.x.

> **Note:** Legacy `mrpt_ros` packages remain released on the ROS build farm for
> already-released ROS distros, but get no new features. New downstream releases
> should target MRPT 3.x directly.

### `package.xml` — dependency renaming

Replace every `<depend>mrpt_lib*</depend>` with the fine-grained module(s)
actually used:

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
| `python_mrpt`       | **Removed** — each module has its own pybind11 bindings |

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

### `CMakeLists.txt` — `find_package` and targets

Same two mechanical rules as the [Build system](#build) section
(`find_package(mrpt-<X>)` → `find_package(mrpt_<X>)`; `mrpt::<X>` →
`mrpt::mrpt_<X>`):

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

### Dropped: `mrpt-tclap` / `mrpt_libtclap`

MRPT 3.x no longer vendors TCLAP. Code using
`#include <mrpt/3rdparty/tclap/CmdLine.h>` must migrate to an external CLI
library. The recommended replacement is **CLI11** (single-header, packaged as
`libcli11-dev` on Ubuntu, rosdep key `cli11`):

```cpp
// MRPT 2.x (TCLAP via the removed mrpt_libtclap):
#include <mrpt/3rdparty/tclap/CmdLine.h>
TCLAP::CmdLine cmd("my tool", ' ', "1.0");
TCLAP::ValueArg<std::string> arg_in("i", "input", "input file", true, "", "file", cmd);
cmd.parse(argc, argv);
const std::string in = arg_in.getValue();

// MRPT 3.0 (CLI11):
#include <CLI/CLI.hpp>
CLI::App app{"my tool"};
std::string in;
app.add_option("-i,--input", in, "input file")->required();
CLI11_PARSE(app, argc, argv);
```
