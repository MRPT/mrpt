\page tutorial_3D_scenes Tutorial: 3D scenes

# 1. Examples

If you prefer directly jumping to example code, see:

- \ref opengl_objects_demo
- \ref opengl_skybox_example
- \ref gui_display3D_example
- \ref gui_nanogui_demo

# 2. MRPT Rendering Architecture

## 2.1 Two-module design: `mrpt_viz` and `mrpt_opengl`

In MRPT 3.0, the 3D scene graph and the OpenGL rendering code live in two
separate modules:

| Module          | Namespace        | Purpose                                             |
|-----------------|------------------|-----------------------------------------------------|
| **mrpt_viz**    | `mrpt::viz`      | Abstract scene graph. No OpenGL/GPU dependencies.   |
| **mrpt_opengl** | `mrpt::opengl`   | GPU rendering, shaders, textures, OpenGL calls.     |

**mrpt_viz** defines all the visual primitives (CBox, CSphere, CGridPlaneXY,
CPointCloud, etc.), the scene container (Scene), viewports (Viewport), and
camera (CCamera). It has virtually no external dependencies beyond MRPT core
libraries, making it usable on systems without GPU or OpenGL.

**mrpt_opengl** takes a `mrpt::viz::Scene` and renders it using OpenGL.
It uses a "compiled scene" approach where abstract viz objects are translated
into GPU-side proxies that own the actual OpenGL buffers (VBOs, VAOs, textures).

## 2.2 Key class hierarchy (mrpt_viz)

```
mrpt::viz::CVisualObject          (base: pose, color, name, visibility, dirty flag)
 ├── VisualObjectParams_Triangles  (mixin: triangle buffer, lighting, cull face)
 ├── VisualObjectParams_TexturedTriangles (mixin: textured triangle buffer + texture image)
 ├── VisualObjectParams_Lines      (mixin: line buffer, line width, anti-aliasing)
 └── VisualObjectParams_Points     (mixin: point buffer, point size)
```

Concrete primitives inherit from CVisualObject and one or more of the
`VisualObjectParams_*` mixins to declare which shader types they need.
For example:
- `CGridPlaneXY` inherits `CVisualObject` + `VisualObjectParams_Lines` (lines only)
- `CSphere` inherits `CVisualObject` + `VisualObjectParams_Triangles` (triangles only)
- `CBox` inherits `CVisualObject` + `VisualObjectParams_Triangles` + `VisualObjectParams_Lines` (both: solid faces + wireframe edges)

The scene graph is:
```
Scene
 └── Viewport[]  ("main" always exists)
      ├── CCamera
      ├── TLightParameters
      └── CVisualObject[]  (the objects to render)
           └── CSetOfObjects  (recursive container)
```

## 2.3 Key class hierarchy (mrpt_opengl) and the Proxy pattern

```
mrpt::opengl::RenderableProxy     (base: compile, updateBuffers, render, requiredShaders)
 ├── PointsProxyBase               → renders VisualObjectParams_Points data
 ├── LinesProxyBase                → renders VisualObjectParams_Lines data
 ├── TrianglesProxyBase            → renders VisualObjectParams_Triangles data
 │    └── TexturedTrianglesProxyBase → adds texture support
 └── (Text3DProxy)                 → internal: generates triangle geometry from CText3D
```

Each `RenderableProxy` owns the GPU resources (buffers, VAO) for one rendering
pass of one viz object. The proxies are created and managed by `CompiledScene`.

The overall rendering pipeline classes are:

```
CompiledScene          manages all proxies, performs incremental updates
 ├── ShaderProgramManager   owns loaded shader programs
 └── CompiledViewport[]     one per viz::Viewport
      ├── RenderableProxy[]  organized by shader ID for batched rendering
      ├── TRenderMatrices    projection/view/model matrix state
      └── FrameBuffer        (optional: shadow map FBO)
```

## 2.4 Rendering data flow

The rendering pipeline has three phases:

### Phase 1: Scene compilation (`CompiledScene::compile()`)
For each `CVisualObject` in the scene:
1. Call `obj->updateBuffers()` — the viz object populates its internal CPU-side
   data buffers (triangles, lines, points) from its geometric parameters.
2. Create a `RenderableProxy` of the appropriate type based on the object's
   `VisualObjectParams_*` mixin(s).
3. Call `proxy->compile(obj)` — the proxy reads the viz object's CPU-side
   buffers and uploads them to GPU (creates VBOs, VAOs).
4. Store `proxy->m_modelMatrix` computed from the object's pose and parent
   transforms.

### Phase 2: Incremental updates (`CompiledScene::updateIfNeeded()`)
Each frame, before rendering:
1. **Cleanup**: Remove proxies whose source objects were deleted (detected via
   `weak_ptr` expiration).
2. **New objects**: Scan the source scene for objects not yet compiled;
   compile them.
3. **Dirty objects**: For each tracked object where `hasToUpdateBuffers()==true`:
   - Call `obj->updateBuffers()` (viz side: regenerate CPU buffers)
   - Call `proxy->updateBuffers(obj)` (opengl side: re-upload to GPU)
   - Call `obj->clearChangedFlag()`

### Phase 3: Rendering (`CompiledViewport::render()`)
For each viewport:
1. Compute pixel viewport from normalized coordinates.
2. Update projection/view matrices from camera.
3. Optionally render shadow map (1st pass into depth FBO).
4. Build a `RenderQueue`: group proxies by shader ID, with depth sorting.
5. Process the queue: for each shader, bind it once, then render all objects
   using that shader (uploading per-object model matrix uniforms).

### Dirty flag mechanism
- User modifies a viz object (e.g., `box.setBoxCorners(...)`)
- The setter calls `notifyChange()`, which sets a per-thread dirty flag
  (`PerThreadDataHolder<OutdatedState>`) to `true` for all threads.
- Before rendering, the GL thread checks `hasToUpdateBuffers()` which reads the
  GL thread's slot — returns `true`.
- After update, `clearChangedFlag()` resets all per-thread slots.

## 2.5 Off-screen rendering with CFBORender

`mrpt::opengl::CFBORender` provides off-screen rendering without a GUI window:
- Creates its own EGL context (headless rendering, works without a display).
- Maintains a `CompiledScene` internally (lazy-initialized on first render).
- Supports RGB and RGB+D output.
- Camera can be overridden via `setCamera()`.

## 2.6 Known limitations and issues (v3.0-dev)

1. **Single proxy per object**: `CompiledScene::createProxyByType()` creates
   only ONE proxy per `CVisualObject`, using a priority order:
   TexturedTriangles > Triangles > Points > Lines. Objects that inherit
   multiple `VisualObjectParams_*` mixins (e.g., `CBox` with both triangles and
   lines for solid faces + wireframe border) will only get a proxy for the
   highest-priority type. The lines/wireframe of such objects will not render.
   **This needs to be fixed** by creating multiple proxies per object when it
   inherits multiple shader parameter mixins.

2. **Model matrix not updated on pose change**: When
   `updateDirtyObjects()` runs, it re-uploads geometry buffers but does NOT
   update `proxy->m_modelMatrix`. If an object's pose (setPose/setLocation)
   changes between compilations, the object renders at its old position.
   The dirty flag IS set by pose changes (via `notifyChange()`), but only the
   geometry buffers are refreshed, not the transform. **This needs to be fixed**
   by recomputing the model matrix during incremental updates.

3. **Cloned viewport handling**: In `CompiledScene::compileViewport()`, the
   early return checks `vizViewport.isClonedCamera()` rather than
   `vizViewport.isCloned()`. A viewport with a cloned camera but its own
   objects would incorrectly skip object compilation.

4. **Frustum culling not implemented**: `CompiledViewport::buildRenderQueue()`
   has a TODO comment: frustum culling is not performed; all objects are
   rendered every frame regardless of visibility.

5. **Depth output double-flip in CFBORender**: The depth buffer is flipped
   twice (once with `colwise().reverse()` and once with a manual row swap
   loop), which cancel each other out. Both flips are unnecessary when
   `flipVerticalProjection(true)` is used (same as RGB path). This is
   wasteful but not a correctness bug.

6. **Texture binding incomplete**: `TexturedTrianglesProxyBase::render()` has
   texture binding code, but the `m_texture` pointer is never set by the
   current `compile()` implementation. Textured objects will render without
   their texture. The `Texture` class needs to be integrated during compilation.

7. **Border rendering unimplemented**: `CompiledViewport::renderBorder()` is
   a stub (TODO comment). Viewport borders are not drawn.

8. **Text overlays not rendered**: The `CTextMessageCapable` text overlay
   system (2D text on top of the viewport) is not yet integrated into the
   compiled rendering pipeline.

# 3. Relevant C++ classes

The main class for building 3D scenes is mrpt::viz::Scene, which allows the
user to create, load, save, and render 3D scenes using predefined 3D entities.

A Scene contains one or more **Viewports** (mrpt::viz::Viewport), each one
associated with a set of visual objects and, optionally, a preferred camera
position. Both orthogonal (2D/3D) and projective camera models can be used for
each viewport independently.

An instance of mrpt::viz::Scene always contains **at least one viewport**
(mrpt::viz::Viewport) named `"main"`, covering the entire window area by default.
If you do not provide a viewport name in API calls, `"main"` will always be
used by default.

Viewports are referenced by their names (case-sensitive strings). Each viewport
contains a different 3D scene (i.e. they render different objects), though a
mechanism exists to share the same 3D scene by a number of viewports so memory
is not wasted replicating the same smart pointers
(see mrpt::viz::Viewport::setCloneView()).

Users will never normally need to invoke rendering manually, but use instead:
- Off-screen rendering: mrpt::opengl::CFBORender
- the 3D standalone viewer: \ref app_SceneViewer3D
- the basic runtime 3D display windows mrpt::gui::CDisplayWindow3D
- the advanced GUI-controls capable display mrpt::gui::CDisplayWindowGUI

An object mrpt::viz::Scene can be saved to a `.3Dscene` file using
mrpt::viz::Scene::saveToFile(), for posterior visualization from
the standalone application [SceneViewer3D](app_SceneViewer3D.html).

# 4. Creating, populating and updating a Scene

Since 3D rendering is performed in a detached thread, special care must be
taken when updating the 3D scene to be rendered. Updating here means either
(i) inserting/removing new primitives to the Scene object associated with a
CDisplayWindow3D, or (ii) modifying the pose, color, contents, etc. of any
of the primitives previously inserted into such Scene object.

To avoid race conditions (rendering always happens on a standalone thread),
the process of updating a 3D scene must make use of a mechanism that
locks/unlocks an internal critical section, and it comprises these steps:

    // Create the GUI window object.
    mrpt::gui::CDisplayWindow3D win("My window", 1024, 800);

    // Lock the 3D scene (**Enter critical section**)
    mrpt::viz::Scene::Ptr &ptrScene = win.get3DSceneAndLock();

    // Modify the scene or the primitives therein:
    ptrScene->...

    // Unlock the 3D scene (**Exit critical section**).
    win.unlockAccess3DScene();

    // Force a window update, if required:
    win.forceRepaint();

An alternative way of updating the scene is by creating,
before locking the 3D window, a new object of class Scene, then locking
the window only for replacing the smart pointer. This may be advantageous
if generating the 3D scene takes a long time, since while the window is locked
it will not be responsive to the user input or window redraw.


# 5. Existing visualization primitives

For a list of existing visualization primitive classes, browse the namespace mrpt::viz,
or inspect the example: \ref opengl_objects_demo

![MRPT opengl primitives](opengl_objects_demo_screenshot.png)

# 6. Text messages

GUI windows offer the possibility of displaying any number of text labels on
top of the rendered scene. Refer to:

- \ref gui_display3D_example
- API: mrpt::gui::CDisplayWindow3D::addTextMessage()

# 7. Advanced UI controls

For more advanced UI controls, including subwindows that can be dragged,
minimized and restored, etc. see:
- mrpt::gui::CDisplayWindowGUI
- \ref gui_nanogui_demo
