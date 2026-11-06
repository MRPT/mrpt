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

#include <mrpt/gui/CBaseGUIWindow.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/gui/CDisplayWindowGUI.h>
#include <mrpt/gui/config.h>
#include <mrpt/img/CImage.h>
#include <mrpt/viz/Scene.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace pybind11::literals;

#if MRPT_HAS_NANOGUI
namespace
{
// nanogui (GLFW) must be initialized once, before creating the first window.
void ensure_nanogui_init()
{
  static bool done = false;
  if (!done)
  {
    nanogui::init();
    done = true;
  }
}

// Python truthiness of a callback result (e.g. None is false), as a C++ bool.
bool py_truthy(const py::object& o)
{
  const int t = PyObject_IsTrue(o.ptr());
  if (t < 0)
  {
    throw py::error_already_set();
  }
  return t == 1;
}

// Python callables must be called with the GIL held, and their exceptions
// must not reach the C++ GUI loop.
void call_py(const py::function& f)
{
  py::gil_scoped_acquire gil;
  try
  {
    f();
  }
  catch (py::error_already_set& e)
  {
    e.discard_as_unraisable("mrpt.gui.CDisplayWindowGUI loop callback");
  }
}
}  // namespace
#endif

PYBIND11_MODULE(_bindings, m)
{
  m.doc() = "Python bindings for mrpt::gui — GUI windows for 3D visualization";

  // -------------------------------------------------------------------------
  // CBaseGUIWindow — abstract base for GUI windows
  // -------------------------------------------------------------------------
  py::class_<mrpt::gui::CBaseGUIWindow, std::shared_ptr<mrpt::gui::CBaseGUIWindow>>(
      m, "CBaseGUIWindow", "The base class for GUI window classes based on wxWidgets.")
      .def(
          "isOpen", &mrpt::gui::CBaseGUIWindow::isOpen,
          "Returns false if the user has already closed the window.")
      .def(
          "waitForKey",
          [](mrpt::gui::CBaseGUIWindow& w, bool ignoreControlKeys)
          { return w.waitForKey(ignoreControlKeys); },
          "ignoreControlKeys"_a = true,
          "Waits until a key is pressed in the window. Returns the key code.")
      .def(
          "keyHit", &mrpt::gui::CBaseGUIWindow::keyHit,
          "Returns true if a key has been pushed, without blocking waiting for a new key being "
          "pushed.")
      .def(
          "clearKeyHitFlag", &mrpt::gui::CBaseGUIWindow::clearKeyHitFlag,
          "Assure that \"keyHit\" will return false until the next pushed key.");

  // -------------------------------------------------------------------------
  // CDisplayWindow3D — interactive 3D OpenGL scene viewer
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::gui::CDisplayWindow3D, mrpt::gui::CBaseGUIWindow,
      std::shared_ptr<mrpt::gui::CDisplayWindow3D>>(
      m, "CDisplayWindow3D",
      "A graphical user interface (GUI) for efficiently rendering 3D scenes in real-time.")
      .def(
          py::init<const std::string&, unsigned int, unsigned int>(), "windowCaption"_a = "",
          "width"_a = 400, "height"_a = 300,
          "Creates and shows a new 3D window with the given caption and size.")
      .def_static(
          "Create",
          [](const std::string& caption, unsigned int w, unsigned int h)
          { return mrpt::gui::CDisplayWindow3D::Create(caption, w, h); },
          "windowCaption"_a = "", "width"_a = 400, "height"_a = 300,
          "Creates a new 3D window (same arguments as the constructor).")
      // Scene access — returns locked scene pointer; must call unlockAccess3DScene() after use
      .def(
          "get3DSceneAndLock",
          [](mrpt::gui::CDisplayWindow3D& w) -> mrpt::viz::Scene::Ptr&
          { return w.get3DSceneAndLock(); },
          py::return_value_policy::reference_internal,
          "Get locked 3D scene pointer. Call unlockAccess3DScene() when done.")
      .def(
          "unlockAccess3DScene", &mrpt::gui::CDisplayWindow3D::unlockAccess3DScene,
          "Releases the scene lock taken by get3DSceneAndLock().")
      .def(
          "forceRepaint", &mrpt::gui::CDisplayWindow3D::forceRepaint,
          "Repaints the window. forceRepaint, repaint and updateWindow are all aliases of the same "
          "method.")
      .def(
          "repaint", &mrpt::gui::CDisplayWindow3D::repaint,
          "Repaints the window. forceRepaint, repaint and updateWindow are all aliases of the same "
          "method.")
      .def(
          "updateWindow", &mrpt::gui::CDisplayWindow3D::updateWindow,
          "Repaints the window. forceRepaint, repaint and updateWindow are all aliases of the same "
          "method.")
      // Camera controls
      .def(
          "setCameraElevationDeg", &mrpt::gui::CDisplayWindow3D::setCameraElevationDeg,
          "Sets the camera elevation angle, in degrees.")
      .def(
          "setCameraAzimuthDeg", &mrpt::gui::CDisplayWindow3D::setCameraAzimuthDeg,
          "Sets the camera azimuth angle, in degrees.")
      .def(
          "setCameraPointingToPoint", &mrpt::gui::CDisplayWindow3D::setCameraPointingToPoint,
          "Sets the point the camera looks at (x, y, z).")
      .def(
          "setCameraZoom", &mrpt::gui::CDisplayWindow3D::setCameraZoom,
          "Sets the camera distance to the point it looks at.")
      .def(
          "setProjectiveModel", &mrpt::gui::CDisplayWindow3D::setProjectiveModel,
          "Sets a perspective (true) or orthographic (false) camera.")
      .def(
          "getCameraElevationDeg", &mrpt::gui::CDisplayWindow3D::getCameraElevationDeg,
          "Returns the camera elevation angle, in degrees.")
      .def(
          "getCameraAzimuthDeg", &mrpt::gui::CDisplayWindow3D::getCameraAzimuthDeg,
          "Returns the camera azimuth angle, in degrees.")
      .def(
          "getCameraPointingToPoint",
          [](const mrpt::gui::CDisplayWindow3D& w)
          {
            float x, y, z;
            w.getCameraPointingToPoint(x, y, z);
            return py::make_tuple(x, y, z);
          },
          "Returns the point the camera looks at, as (x, y, z).")
      .def(
          "getCameraZoom", &mrpt::gui::CDisplayWindow3D::getCameraZoom,
          "Returns the camera distance to the point it looks at.")
      .def(
          "isCameraProjective", &mrpt::gui::CDisplayWindow3D::isCameraProjective,
          "Returns true for a perspective camera, false for an orthographic one.")
      .def(
          "useCameraFromScene", &mrpt::gui::CDisplayWindow3D::useCameraFromScene,
          "If true, the camera of the scene viewport is used instead of the mouse-controlled one.")
      .def(
          "setFOV", &mrpt::gui::CDisplayWindow3D::setFOV,
          "Sets the camera field of view, in degrees.")
      .def(
          "getFOV", &mrpt::gui::CDisplayWindow3D::getFOV,
          "Returns the camera field of view, in degrees.")
      .def(
          "setMinRange", &mrpt::gui::CDisplayWindow3D::setMinRange,
          "Sets the near clip distance of the camera.")
      .def(
          "setMaxRange", &mrpt::gui::CDisplayWindow3D::setMaxRange,
          "Sets the far clip distance of the camera.")
      // Window management
      .def(
          "resize",
          [](mrpt::gui::CDisplayWindow3D& w, unsigned int width, unsigned int height)
          { w.resize(width, height); },
          "Resizes the window, stretching the image to fit into the display area.")
      .def(
          "setPos", [](mrpt::gui::CDisplayWindow3D& w, int x, int y) { w.setPos(x, y); },
          "Changes the position of the window on the screen.")
      .def(
          "setWindowTitle",
          [](mrpt::gui::CDisplayWindow3D& w, const std::string& t) { w.setWindowTitle(t); },
          "Changes the window title.")
      .def(
          "getRenderingFPS", &mrpt::gui::CDisplayWindow3D::getRenderingFPS,
          "Get the average Frames Per Second (FPS) value from the last 250 rendering events.")
      // Image display
      .def(
          "setImageView",
          [](mrpt::gui::CDisplayWindow3D& w, const mrpt::img::CImage& img) { w.setImageView(img); },
          "img"_a, "Display a 2D image in this window")
      // Image grabbing
      .def(
          "grabImagesStart", &mrpt::gui::CDisplayWindow3D::grabImagesStart, "prefix"_a = "video_",
          "Starts saving each rendered frame as a PNG file, with the given filename prefix.")
      .def(
          "grabImagesStop", &mrpt::gui::CDisplayWindow3D::grabImagesStop,
          "Stops image grabbing started by grabImagesStart.")
      .def(
          "__repr__",
          [](const mrpt::gui::CDisplayWindow3D& w)
          {
            return "CDisplayWindow3D(open=" +
                   std::string(
                       const_cast<mrpt::gui::CDisplayWindow3D&>(w).isOpen() ? "True" : "False") +
                   ")";
          });

#if MRPT_HAS_NANOGUI
  // -------------------------------------------------------------------------
  // CDisplayWindowGUI: nanogui (GLFW) window, with a background 3D scene
  // -------------------------------------------------------------------------
  using mrpt::gui::CDisplayWindowGUI;
  using mrpt::gui::CDisplayWindowGUI_Params;

  py::class_<CDisplayWindowGUI_Params>(
      m, "CDisplayWindowGUI_Params", "Additional parameters for CDisplayWindowGUI windows.")
      .def(py::init<>())
      .def_readwrite("resizable", &CDisplayWindowGUI_Params::resizable)
      .def_readwrite("fullscreen", &CDisplayWindowGUI_Params::fullscreen)
      .def_readwrite("maximized", &CDisplayWindowGUI_Params::maximized)
      .def_readwrite(
          "nSamples", &CDisplayWindowGUI_Params::nSamples,
          "Number of MSAA samples (0: no antialiasing).");

  py::class_<CDisplayWindowGUI, std::shared_ptr<CDisplayWindowGUI>>(
      m, "CDisplayWindowGUI",
      "A window with a background 3D scene and mouse camera control, based on nanogui.\n\n"
      "Set the scene to show in `background_scene`. Then, either call spinOnce() from your\n"
      "own loop (it does not block), or register callbacks with addLoopCallback() and call\n"
      "mrpt.gui.nanogui_mainloop(), which blocks until all windows are closed.")
      .def(
          py::init(
              [](const std::string& caption, unsigned int width, unsigned int height,
                 const CDisplayWindowGUI_Params& p)
              {
                ensure_nanogui_init();
                auto w = std::make_shared<CDisplayWindowGUI>(caption, width, height, p);
                w->performLayout();
                w->drawAll();
                w->setVisible(true);
                return w;
              }),
          "caption"_a = "", "width"_a = 800, "height"_a = 600,
          "params"_a = CDisplayWindowGUI_Params(), "Creates and shows a new window.")
      .def_property(
          "background_scene",
          [](CDisplayWindowGUI& w)
          {
            std::lock_guard<std::mutex> lck(w.background_scene_mtx);
            return w.background_scene;
          },
          [](CDisplayWindowGUI& w, const mrpt::viz::Scene::Ptr& scene)
          {
            std::lock_guard<std::mutex> lck(w.background_scene_mtx);
            w.background_scene = scene;
          },
          "The 3D scene rendered as the window background (None: no scene).")
      .def(
          "camera",
          [](CDisplayWindowGUI& w) -> mrpt::viz::COrbitCameraController& { return w.camera(); },
          py::return_value_policy::reference_internal,
          "The mouse-controlled camera (a mrpt.viz.COrbitCameraController).")
      .def(
          "spinOnce",
          [](CDisplayWindowGUI& w)
          {
            py::gil_scoped_release nogil;
            return w.spinOnce();
          },
          "Processes pending window events and repaints the window, without blocking.\n"
          "Returns False once the window has been closed.")
      .def(
          "isOpen",
          [](CDisplayWindowGUI& w)
          { return w.visible() && !glfwWindowShouldClose(w.glfwWindow()); },
          "Returns False if the window has been closed.")
      .def(
          "setVisible", [](CDisplayWindowGUI& w, bool v) { w.setVisible(v); }, "visible"_a,
          "Shows or hides the window.")
      .def("resize", &CDisplayWindowGUI::resize, "width"_a, "height"_a, "Resizes the window.")
      .def(
          "setPos", &CDisplayWindowGUI::setPos, "x"_a, "y"_a,
          "Changes the position of the window on the screen.")
      .def(
          "setWindowTitle", &CDisplayWindowGUI::setWindowTitle, "title"_a,
          "Changes the window title.")
      .def(
          "setIcon", &CDisplayWindowGUI::setIcon, "img"_a,
          "Sets the window icon from an RGB or RGBA image.")
      .def(
          "addLoopCallback",
          [](CDisplayWindowGUI& w, const py::function& f)
          { w.addLoopCallback([f]() { call_py(f); }); },
          "callback"_a, "Adds a function to call before each repaint of the window.")
      .def(
          "addKeyboardCallback",
          [](CDisplayWindowGUI& w, const py::function& f)
          {
            w.addKeyboardCallback(
                [f](int key, int scancode, int action, int modifiers)
                {
                  py::gil_scoped_acquire gil;
                  try
                  {
                    return py_truthy(f(key, scancode, action, modifiers));
                  }
                  catch (py::error_already_set& e)
                  {
                    e.discard_as_unraisable("mrpt.gui.CDisplayWindowGUI keyboard callback");
                  }
                  return false;
                });
          },
          "callback"_a,
          "Adds a keyboard handler f(key, scancode, action, modifiers) -> bool (True if handled).\n"
          "Key codes are GLFW key codes (e.g. ord('A') for letters, 256 for ESC).")
      .def(
          "addDropFilesCallback",
          [](CDisplayWindowGUI& w, const py::function& f)
          {
            w.addDropFilesCallback(
                [f](const std::vector<std::string>& files)
                {
                  py::gil_scoped_acquire gil;
                  try
                  {
                    return py_truthy(f(files));
                  }
                  catch (py::error_already_set& e)
                  {
                    e.discard_as_unraisable("mrpt.gui.CDisplayWindowGUI drop files callback");
                  }
                  return false;
                });
          },
          "callback"_a, "Adds a handler f(filenames) -> bool for files dropped onto the window.")
      .def(
          "__repr__",
          [](CDisplayWindowGUI& w) {
            return std::string("CDisplayWindowGUI(visible=") + (w.visible() ? "True" : "False") +
                   ")";
          });

  m.def(
      "nanogui_mainloop",
      [](int idleLoopPeriod, int minRepaintPeriod)
      {
        ensure_nanogui_init();
        py::gil_scoped_release nogil;
        nanogui::mainloop(idleLoopPeriod, minRepaintPeriod);
      },
      "idle_loop_period_ms"_a = 50, "min_repaint_period_ms"_a = 50,
      "Runs the GUI loop of all CDisplayWindowGUI windows until they are all closed, or\n"
      "nanogui_leave() is called. Loop callbacks run before each repaint.");
  m.def(
      "nanogui_leave", []() { nanogui::leave(); },
      "Makes nanogui_mainloop() return (e.g. call it from a callback).");
#endif

  m.attr("HAS_NANOGUI") = py::bool_(MRPT_HAS_NANOGUI != 0);
  m.attr("HAS_WXWIDGETS") = py::bool_(MRPT_HAS_WXWIDGETS != 0);
}
