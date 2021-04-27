\page tutorial_3D_scenes Tutorial: 3D scenes

# 1. Examples

If you prefer directly jumping to example code, see:

- \ref opengl_objects_demo
- \ref gui_display3D_example
- \ref gui_nanogui_demo

# 2. Relevant C++ classes

The main class behind 3D scenes is mrpt::opengl::COpenGLScene, which allows the user to create, load, save, and render 3D scenes using predefined 3D entities.
The class can be understood as a program to be run on the OpenGL system,
containing a sequence of viewport definitions, primitive objects, shaders, etc...

An "OpenGL scene" contains from 1 up to any number of **Viewports**, each one
associated a set of visual objects and, optionally, a preferred camera position.
Both orthogonal (2D/3D) and projective camera models can be used for each
viewport independently, greatly increasing the possibilities of rendered scenes.
An instance of mrpt::opengl::COpenGLScene always contains **at least one viewport** (mrpt::opengl::COpenGLViewport), named `"main"`, covering the entire window area by default.
If you do not provide a viewport name in API calls, `"main"` will be always be used by default,
so do not worry on remembering that name.

Viewports are referenced by their names, case-sensitive strings. Each viewport contains a different
"3D scene" (i.e. they render different objects), though a mechanism exists to share the same 3D scene by a number of viewports so memory is not wasted replicating the same smart pointers (see mrpt::opengl::COpenGLViewport::setCloneView()).

The main rendering method, COpenGLScene::render(), assumes that a viewport has been set-up for the entire target window. That method will internally make the required calls to opengl for creating the additional viewports. Note that only the depth buffer is cleared by default for each (non-main) viewport, to allow transparencies. This can be disabled by the approppriate member in COpenGLViewport.

Users will never normally need to invoke COpenGLScene::render() manually, but use instead:
- the 3D standalone viewer: [SceneViewer3D](app_SceneViewer3D.html)
- the basic runtime 3D display windows mrpt::gui::CDisplayWindow3D
- the advanced GUI-controls capable display  mrpt::gui::CDisplayWindowGUI
- wxWidgets / Qt controls also exist for integration into custom GUIs (see mrpt::gui:: mrpt::gui::CGlCanvasBase)

An object mrpt::opengl::COpenGLScene can be saved to a `.3Dscene` file using 
mrpt::opengl::COpenGLScene::saveToFile(), for posterior visualization from
the standalone application [SceneViewer3D](app_SceneViewer3D.html).

# 3. Creating, populating and updating a COpenGLScene

Since 3D rendering is performed in a detached thread, especial care must be taken when updating the 3D scene to be rendered. Updating here means either (i) inserting/removing new primitives to the COpenGLScene object associated with a CDisplayWindow3D, or (ii) modifying the pose, color, contents, etc. of any of the primitives previously inserted into such COpenGLScene object.

To avoid rance conditions (rendering always happens on an standalone thread),
the process of updating a 3D scene must make use of a mechanism that
locks/unlocks an internal critical section, and it comprises these steps:

    // Create the GUI window object.
    // This will lock until the detached MRPT GUI thread creates
    // the window successfully.
    mrpt::gui::CDisplayWindow3D win("My window", 1024, 800);

    // Lock the 3D scene (**Enter critical section**)
    // get3DSceneAndLock() must be called to prevent the rendering
    // thread accessing the OpenGLScene while we are manipulating its contents.
    // This function returns a reference to a smart pointer which may be used for:
    // a) Create mrpt::opengl primitives and insert them into the scene. This is
    //    the typical first use of get3DSceneAndLock().
    // b) Just ignore it. In subsequent calls, if the scene was already populated
    //    and you don't need to search for any primitive object (e.g. because you
    //    kept a copy of all mrpt::opengl::XXX::Ptr pointers for direct access),
    //    just ignore the return value.
    // c) Replace the entire 3D scene (read below after the code snippet)
    mrpt::opengl::COpenGLScene::Ptr &ptrScene = win.get3DSceneAndLock();

    // Modify the scene or the primitives therein:
    ptrScene->...

    // Unlock the 3D scene (**Exit critical section**).
    // Required for the window to be able to be redrawn.
    win.unlockAccess3DScene();

    // Force a window update, if required:
    win.forceRepaint();

An alternative way of updating the scene is by creating,
before locking the 3D window, a new object of class COpenGLScene, then locking
the window only for replacing the smart pointer. This may be advantageous
is generating the 3D scene takes a long time, since while the window is locked
it will not be responsive to the user input or window redraw.


# 4. Existing visualization primitives

For a list of existing visualization primitive classes, browse the namespace mrpt::opengl,
or inspect the example: \ref opengl_objects_demo

![MRPT opengl primitives](opengl_objects_demo_screenshot.png)

# 5. Text messages

GUI windows offer the possibility of displaying any number of text labels on
top of the rendered scene. Refer to:

- \ref gui_display3D_example
- API: mrpt::gui::CDisplayWindow3D::addTextMessage()

# 6. Advanced UI controls

For more advanced UI controls, including subwindows that can be dragged,
minimized and restored, etc. see:
- mrpt::gui::CDisplayWindowGUI
- \ref gui_nanogui_demo
