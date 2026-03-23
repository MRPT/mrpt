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


### 13.6 `mrpt_nav` — Comprehensive modernisation plan

The `mrpt_nav` module is ~20 years old and the largest candidate for
refactoring in MRPT 3.0.  The plan below is grouped into phases of
increasing risk/effort.  Each item is prefixed with a priority tag:
**[P0]** = do first (low risk, high value),
**[P1]** = next (medium risk),
**[P2]** = later (significant API change).

---

#### 13.6.1 Enum modernisation

**[DONE]** Converted all plain `enum` to `enum class`, with redundant prefixes removed from values:

| Old | New |
|-----|-----|
| `CAbstractNavigator::TState` — `IDLE`, `NAVIGATING`, `SUSPENDED`, `NAV_ERROR` | `enum class TState` — same values (`NAV_ERROR` kept to avoid clash with Windows `ERROR` macro) |
| `CAbstractNavigator::TErrorCode` — `ERR_NONE`, `ERR_EMERGENCY_STOP`, `ERR_CANNOT_REACH_TARGET`, `ERR_OTHER` | `enum class TErrorCode` — `NONE`, `EMERGENCY_STOP`, `CANNOT_REACH_TARGET`, `OTHER` |
| `CHolonomicND::TSituations` — `SITUATION_TARGET_DIRECTLY`, `SITUATION_SMALL_GAP`, `SITUATION_WIDE_GAP`, `SITUATION_NO_WAY_FOUND` | `enum class TSituations` — `TARGET_DIRECTLY`, `SMALL_GAP`, `WIDE_GAP`, `NO_WAY_FOUND` |
| `PTG_collision_behavior_t` — `COLL_BEH_BACK_AWAY`, `COLL_BEH_STOP` | `enum class PTGCollisionBehavior` — `BACK_AWAY`, `STOP` |

---

#### 13.6.2 `[[nodiscard]]` annotations

**[DONE]** Added `[[nodiscard]]` to:

- `CAbstractNavigator`: `getCurrentState()`, `getErrorReason()`,
  `getFrameTF()`, `isRethrowNavExceptionsEnabled()`, `getDelaysTimeLogger()`
- `CParameterizedTrajectoryGenerator`: `getDescription()`, `supportVelCmdNOP()`,
  `supportSpeedAtTarget()`, `isInitialized()`, `getAlphaValuesCount()`,
  `getPathCount()`, `getRefDistance()`, `getScorePriority()`,
  `getClearanceStepCount()`, `getClearanceDecimatedPaths()`, `getPathStepCount()`,
  `getPathPose()`, `getPathDist()`, `getPathStepDuration()`, `getMaxLinVel()`,
  `getMaxAngVel()`, `getMaxRobotRadius()`, `isPointInsideRobotShape()`
- `CAbstractHolonomicReactiveMethod`: `getAssociatedPTG()`
- `CWaypointsNavigator`: `getWaypointsAccessGuard()`

Still pending: `CAbstractPTGBasedReactive` getters, holonomic `TOptions` getters.

---

#### 13.6.3 Naming consistency

**[PARTIAL]** Done:
- `CPTG_DiffDrive_CC/CCS/alpha`: replaced `os::sprintf` + `(int)K` casts with
  `mrpt::format()` + `static_cast<int>(K)`.

Remaining:
- **Config structs**: Holonomic classes use `TOptions`; navigator classes use `TParams`.
  Pick one suffix uniformly.
- **STEP-numbered methods**: `STEP1_InitPTGs()` … `STEP8_GenerateLogRecord()` —
  rename to descriptive names.
- **`impl_` prefix**: Standardise virtual hook naming convention.

---

#### 13.6.4 Thread-safety fixes

**[DONE]**:
- `CAbstractPTGBasedReactive::preDestructor()`: replaced bare
  `m_nav_cs.lock(); m_nav_cs.unlock()` with `{ std::scoped_lock lck(m_nav_cs); }`.
- `CWaypointsNavigator`: replaced `beginWaypointsAccess()` / `endWaypointsAccess()`
  with `getWaypointsAccessGuard()` returning a `WaypointsAccessGuard` RAII object
  that holds `std::unique_lock<std::recursive_mutex>` and exposes `waypoints()`.

Remaining: global mutable `OUTPUT_DEBUG_PATH_PREFIX` / `COLLISION_BEHAVIOR` statics
have no synchronisation — consider moving to per-instance fields or guarding with mutex.

---

#### 13.6.5 Return-value modernisation (output-reference cleanup)

**[TODO — P1]** Key candidates:

| Method | Current | Proposed |
|--------|---------|----------|
| `CRobot2NavInterface::getCurrentPoseAndSpeeds()` | 4 output `&` params | Return `struct PoseAndSpeeds { TPose2D pose; TTwist2D vel; … };` |
| `CAbstractHolonomicReactiveMethod::navigate()` | `NavOutput&` | Return `NavOutput` by value |
| `CMultiObjectiveMotionOptimizerBase::decide()` | `int&` best index | Return `std::optional<size_t>` (nullopt = no good motion) |
| `CParameterizedTrajectoryGenerator::inverseMap_WS2TP()` | `int& k, double& d` output | Return `std::optional<std::pair<uint16_t,double>>` |

---

#### 13.6.6 Raw-pointer cleanup

**[DONE — documented]**: Both `m_associatedPTG` in `CAbstractHolonomicReactiveMethod`
and `TCandidateMovementPTG::PTG` are non-owning observer raw pointers. Both are now
documented explicitly as such, with lifetime contract stated. Since `nullptr` is a valid
state and the PTG lifetime is guaranteed by the owning reactive system, a raw pointer
is the correct tool; `std::weak_ptr` would require the entire PTG ownership chain to
be changed to shared ownership.

Remaining: `CAbstractPTGBasedReactive::getHoloMethod()` returns a raw pointer;
consider returning `CAbstractHolonomicReactiveMethod&` (with assert) for non-nullable
use sites.

---

#### 13.6.7 Decompose `CAbstractPTGBasedReactive`

**[P2]** This 467-line header (11 virtual methods, 6 nested structs,
private members spanning PTG management, holonomic method management,
logging, obstacle processing, velocity generation, and profiling) is the
single largest maintenance burden.  Proposed decomposition:

1. **Extract `PTGSetManager`** — owns the `vector<CParameterizedTrajectoryGenerator::Ptr>`,
   handles `STEP1_InitPTGs()`, `getPTG()`, `getPTG_count()`,
   collision-grid builds.
2. **Extract `NavigationLogger`** — owns the `CLogFileRecord`,
   `m_critZoneLastLog`, log-file path management, `STEP8_GenerateLogRecord()`.
3. **Extract `VelocityFilter`** — owns `TSentVelCmd`, the speed-filter
   tau, and `filterVelocityCommand()`.
4. Keep the main class as a thin orchestrator that wires the above
   together via dependency injection (constructor parameters or setters).

---

#### 13.6.8 Refactor long functions

**[P2]** Several functions exceed 200 lines and mix multiple levels of
abstraction.  Each should be broken into named helpers:

| Function | Lines | Suggested split |
|----------|-------|-----------------|
| `PlannerSimple2D::computePath()` | ~465 | Extract `wavePropagation()`, `backtrackPath()` |
| `PlannerRRT_SE2_TPS::solve()` | ~380 | Extract `sampleFreeSpace()`, `extendTree()`, `optimizePath()` |
| `CHolonomicFullEval::evalSingleTarget()` | ~290 | Extract `scoreCandidateDirection()`, `applyPenalties()` |
| `CHolonomicFullEval::navigate()` | ~170 | Extract `findGaps()`, `selectBestDirection()` |
| `CHolonomicND::gapsEstimator()` | ~150 | Extract gap-merging and gap-filtering helpers |
| `CLogFileRecord::serializeFrom()` | ~450 | Version-switch per struct field group |

---

#### 13.6.9 Magic numbers → named constants

**[DONE]**
- `CHolonomicVFF.cpp`: `1e6` → `MAX_FORCE_CAP`, `20.0` → `OBSTACLE_WEIGHT_FACTOR`,
  `6.0` → `OBSTACLE_NEARNESS_THRESHOLD`.
- `CHolonomicND.cpp`: `0.02` → `DIRECT_PATH_SECTOR_FRACTION`,
  `1.05` → `DIRECT_PATH_DIST_MARGIN`, `0.95` → `DIRECT_PATH_RANGE_FRACTION`,
  `0.01` → `MIN_TARGET_DIST`.
- `CHolonomicFullEval.cpp` (`evalSingleTarget`): `1.02` → `OBSTACLE_PAST_TARGET_MARGIN`,
  `0.95` → `TARGET_DIST_SAFETY_FRACTION`, `1.05` → `TARGET_CLEARANCE_MARGIN`,
  `1.01` → `SQRT_DOMAIN_EPS`, `0.1` → `CLEARANCE_WINDOW_HALF_FRACTION`,
  `0.99` → `FREE_SPACE_THRESHOLD`, `4.0` → `SECTOR_DIST_WEIGHT`,
  `0.1` → `BLOCKED_DIR_SCORE_PENALTY`.

---

#### 13.6.10 Test coverage

**[PARTIAL]** Added `tests/holonomic_unittest.cpp` covering:

- `CHolonomicVFF` in isolation (5 tests: heading accuracy × 3, speed
  bounds, slow-down near target, behaviour with blocked obstacles).
- `CHolonomicND` in isolation (4 tests: heading accuracy × 2, speed
  bounds, detour when target direction blocked).
- `CHolonomicFullEval` in isolation (5 tests: heading accuracy × 3, speed
  bounds, repeated calls / hysteresis stability). Also fixed two latent
  bugs: default `TOptions` had 7 factorWeights for `NUM_FACTORS=8`, and
  factor[7] (heading) dereferenced `ptg` without a null guard.
- `ClearanceDiagram` (4 tests: construction, resize, clear, resize-twice).

Still zero dedicated tests for:

- `CWaypointsNavigator` / `TWaypointSequence` state machine.
- `CNavigatorManualSequence`.
- `PlannerRRT_SE2_TPS`, `PlannerRRT_common`.
- `CMultiObjectiveMotionOptimizerBase`, `CMultiObjMotionOpt_Scalarization`.

---

#### 13.6.11 Documentation

**[DONE]** `lib_mrpt_nav.md` expanded to a full reference including:
navigator state machine, waypoint sequencing (RAII guard API), TP-Space
reactive core, holonomic methods table, PTG class table, robot interface
guide, and path planning overview.

---

#### 13.6.12 Remaining code-quality items

**[DONE]** Quick wins completed:

- `MRPT_TODO("Optimize getNearestNode() with KD-tree!")` in
  `PlannerRRT_SE2_TPS.cpp` — converted to plain `// TODO:` comment.
- `static size_t SAVE_LOG_SOLVE_COUNT` — kept (used to name debug log
  files across solve() calls, guarded by `params.save_3d_log_freq > 0`).
- `CLogFileRecord::serializeFrom()` raw `new` — replaced with
  `std::make_shared` (three sites).
- C-style casts `(int)K` — replaced with `static_cast<int>(K)` in
  `CPTG_DiffDrive_CC.cpp`, `CPTG_DiffDrive_CCS.cpp`, `CPTG_DiffDrive_alpha.cpp`.

### 13.7 `mrpt_maps` — Miscellaneous

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
| `lib_mrpt_nav.md` | **[DONE]** — expanded with architecture diagram, state machine tables, PTG/holonomic class tables, waypoint API, robot interface guide |
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
