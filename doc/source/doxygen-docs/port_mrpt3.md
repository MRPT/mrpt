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

- `mrpt::vision` — Removed; will move computer vision features to a new package `mola_vision`.
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

## 10. Moved classes/functions

- `mrpt::vision::CVideoFileWriter.h` becomes `mrpt::img::CVideoFileWriter.h`. It no longer supports MP4, only simpler AVI formats, but it is selfcontained without external libraries.
- `mrpt::hwdrivers::prepareVideoSourceFromUserSelection()` becomes `mrpt::apps::prepareVideoSourceFromUserSelection()` (header: `<mrpt/apps_gui/CameraSelectionGUI.h>`).

When using this function, add `mrpt_libapps_gui` to your CMake dependencies.

---

## 11. Step-by-step migration checklist

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
