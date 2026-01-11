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

// pybind11
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

// MRPT headers
#include <mrpt/viz/CAssimpModel.h>
#include <mrpt/viz/CCamera.h>
#include <mrpt/viz/CPointCloud.h>
#include <mrpt/viz/CPointCloudColoured.h>
#include <mrpt/viz/CSetOfObjects.h>
#include <mrpt/viz/CVisualObject.h>
#include <mrpt/viz/Scene.h>
#include <mrpt/viz/Viewport.h>

namespace py = pybind11;
using namespace mrpt::viz;
using namespace pybind11::literals;

PYBIND11_MODULE(_bindings, m)
{
  // 1. Base Class: CVisualObject
  py::class_<CVisualObject, std::shared_ptr<CVisualObject>>(m, "CVisualObject")
      .def_property("name", &CVisualObject::getName, &CVisualObject::setName)
      .def_property(
          "pose", &CVisualObject::getPose,
          static_cast<CVisualObject& (CVisualObject::*)(const mrpt::poses::CPose3D&)>(
              &CVisualObject::setPose))
      .def_property("visible", &CVisualObject::isVisible, &CVisualObject::setVisibility)
      .def(
          "setColor",
          [](CVisualObject& self, uint8_t r, uint8_t g, uint8_t b, uint8_t a)
          { self.setColor(mrpt::img::TColorf(mrpt::img::TColor(r, g, b, a))); },
          "r"_a, "g"_a, "b"_a, "a"_a = 255);

  // 2. CSetOfObjects (The container node)
  py::class_<CSetOfObjects, CVisualObject, std::shared_ptr<CSetOfObjects>>(m, "CSetOfObjects")
      .def(py::init<>())
      .def(
          "insert",
          static_cast<void (CSetOfObjects::*)(const CVisualObject::Ptr&)>(&CSetOfObjects::insert),
          "obj"_a)
      .def("clear", &CSetOfObjects::clear)
      .def("__len__", [](const CSetOfObjects& self) { return self.size(); });

  // 3. Viewport
  py::class_<Viewport, std::shared_ptr<Viewport>>(m, "Viewport")
      .def_property_readonly("name", &Viewport::getName)
      .def("insert", &Viewport::insert, "obj"_a)
      .def("clear", &Viewport::clear)
      .def("setViewportPosition", &Viewport::setViewportPosition)
      .def("setCustomBackgroundColor", &Viewport::setCustomBackgroundColor)
      .def(
          "getCamera", static_cast<CCamera& (Viewport::*)()>(&Viewport::getCamera),
          py::return_value_policy::reference_internal);

  // 4. Scene (Top-level)
  py::class_<Scene, std::shared_ptr<Scene>>(m, "Scene")
      .def(py::init<>())
      .def(
          "getViewport", &Scene::getViewport, "name"_a = std::string("main"),
          py::return_value_policy::reference_internal)
      .def("createViewport", &Scene::createViewport, "name"_a)
      .def("clear", &Scene::clear)
      .def(
          "insert",
          [](Scene& self, const CVisualObject::Ptr& obj) { self.getViewport()->insert(obj); });

  // 5. CCamera
  py::class_<CCamera, CVisualObject, std::shared_ptr<CCamera>>(m, "CCamera")
      .def(py::init<>())
      .def("setAzimuthDegrees", &CCamera::setAzimuthDegrees)
      .def("setElevationDegrees", &CCamera::setElevationDegrees)
      .def("setZoomDistance", &CCamera::setZoomDistance);

  // 6. CPointCloud
  py::class_<CPointCloud, CVisualObject, std::shared_ptr<CPointCloud>>(m, "CPointCloud")
      .def(py::init<>())
      .def("clear", &CPointCloud::clear)
      .def(
          "insertPoint",
          static_cast<void (CPointCloud::*)(float, float, float)>(&CPointCloud::insertPoint), "x"_a,
          "y"_a, "z"_a)
      .def(
          "setPoints",
          [](CPointCloud& self, const py::array_t<float>& pts)
          {
            auto r = pts.unchecked<2>();
            self.clear();
            for (ssize_t i = 0; i < r.shape(0); i++)
            {
              self.insertPoint(r(i, 0), r(i, 1), r(i, 2));
            }
          });

  // 7. CAssimpModel
  py::class_<CAssimpModel, CVisualObject, std::shared_ptr<CAssimpModel>>(m, "CAssimpModel")
      .def(py::init<>())
      .def(
          "loadScene", &CAssimpModel::loadScene, py::arg("file_name"),
          py::arg("flags") =
              (CAssimpModel::LoadFlags::RealTimeMaxQuality | CAssimpModel::LoadFlags::FlipUVs |
               CAssimpModel::LoadFlags::Verbose));

  py::enum_<CAssimpModel::LoadFlags::flags_t>(m, "AssimpLoadFlags", py::arithmetic())
      .value("RealTimeFast", CAssimpModel::LoadFlags::RealTimeFast)
      .value("RealTimeQuality", CAssimpModel::LoadFlags::RealTimeQuality)
      .value("RealTimeMaxQuality", CAssimpModel::LoadFlags::RealTimeMaxQuality)
      .value("FlipUVs", CAssimpModel::LoadFlags::FlipUVs)
      .value("IgnoreMaterialColor", CAssimpModel::LoadFlags::IgnoreMaterialColor)
      .value("Verbose", CAssimpModel::LoadFlags::Verbose)
      .export_values();
}