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
#include <mrpt/img/CImage.h>
#include <mrpt/viz/Scene.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace pybind11::literals;

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
}
