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
      m, "CBaseGUIWindow")
      .def("isOpen", &mrpt::gui::CBaseGUIWindow::isOpen)
      .def(
          "waitForKey",
          [](mrpt::gui::CBaseGUIWindow& w, bool ignoreControlKeys)
          { return w.waitForKey(ignoreControlKeys); },
          "ignoreControlKeys"_a = true,
          "Waits until a key is pressed in the window. Returns the key code.")
      .def("keyHit", &mrpt::gui::CBaseGUIWindow::keyHit)
      .def("clearKeyHitFlag", &mrpt::gui::CBaseGUIWindow::clearKeyHitFlag);

  // -------------------------------------------------------------------------
  // CDisplayWindow3D — interactive 3D OpenGL scene viewer
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::gui::CDisplayWindow3D, mrpt::gui::CBaseGUIWindow,
      std::shared_ptr<mrpt::gui::CDisplayWindow3D>>(m, "CDisplayWindow3D")
      .def(
          py::init<const std::string&, unsigned int, unsigned int>(), "windowCaption"_a = "",
          "width"_a = 400, "height"_a = 300)
      .def_static(
          "Create",
          [](const std::string& caption, unsigned int w, unsigned int h)
          { return mrpt::gui::CDisplayWindow3D::Create(caption, w, h); },
          "windowCaption"_a = "", "width"_a = 400, "height"_a = 300)
      // Scene access — returns locked scene pointer; must call unlockAccess3DScene() after use
      .def(
          "get3DSceneAndLock",
          [](mrpt::gui::CDisplayWindow3D& w) -> mrpt::viz::Scene::Ptr&
          { return w.get3DSceneAndLock(); },
          py::return_value_policy::reference_internal,
          "Get locked 3D scene pointer. Call unlockAccess3DScene() when done.")
      .def("unlockAccess3DScene", &mrpt::gui::CDisplayWindow3D::unlockAccess3DScene)
      .def("forceRepaint", &mrpt::gui::CDisplayWindow3D::forceRepaint)
      .def("repaint", &mrpt::gui::CDisplayWindow3D::repaint)
      .def("updateWindow", &mrpt::gui::CDisplayWindow3D::updateWindow)
      // Camera controls
      .def("setCameraElevationDeg", &mrpt::gui::CDisplayWindow3D::setCameraElevationDeg)
      .def("setCameraAzimuthDeg", &mrpt::gui::CDisplayWindow3D::setCameraAzimuthDeg)
      .def("setCameraPointingToPoint", &mrpt::gui::CDisplayWindow3D::setCameraPointingToPoint)
      .def("setCameraZoom", &mrpt::gui::CDisplayWindow3D::setCameraZoom)
      .def("setCameraProjective", &mrpt::gui::CDisplayWindow3D::setCameraProjective)
      .def("getCameraElevationDeg", &mrpt::gui::CDisplayWindow3D::getCameraElevationDeg)
      .def("getCameraAzimuthDeg", &mrpt::gui::CDisplayWindow3D::getCameraAzimuthDeg)
      .def(
          "getCameraPointingToPoint",
          [](const mrpt::gui::CDisplayWindow3D& w)
          {
            float x, y, z;
            w.getCameraPointingToPoint(x, y, z);
            return py::make_tuple(x, y, z);
          })
      .def("getCameraZoom", &mrpt::gui::CDisplayWindow3D::getCameraZoom)
      .def("isCameraProjective", &mrpt::gui::CDisplayWindow3D::isCameraProjective)
      .def("useCameraFromScene", &mrpt::gui::CDisplayWindow3D::useCameraFromScene)
      .def("setFOV", &mrpt::gui::CDisplayWindow3D::setFOV)
      .def("getFOV", &mrpt::gui::CDisplayWindow3D::getFOV)
      .def("setMinRange", &mrpt::gui::CDisplayWindow3D::setMinRange)
      .def("setMaxRange", &mrpt::gui::CDisplayWindow3D::setMaxRange)
      // Window management
      .def(
          "resize", [](mrpt::gui::CDisplayWindow3D& w, unsigned int width, unsigned int height)
          { w.resize(width, height); })
      .def("setPos", [](mrpt::gui::CDisplayWindow3D& w, int x, int y) { w.setPos(x, y); })
      .def(
          "setWindowTitle",
          [](mrpt::gui::CDisplayWindow3D& w, const std::string& t) { w.setWindowTitle(t); })
      .def("getRenderingFPS", &mrpt::gui::CDisplayWindow3D::getRenderingFPS)
      // Image display
      .def(
          "setImageView",
          [](mrpt::gui::CDisplayWindow3D& w, const mrpt::img::CImage& img) { w.setImageView(img); },
          "img"_a, "Display a 2D image in this window")
      // Image grabbing
      .def("grabImagesStart", &mrpt::gui::CDisplayWindow3D::grabImagesStart, "prefix"_a = "video_")
      .def("grabImagesStop", &mrpt::gui::CDisplayWindow3D::grabImagesStop)
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
