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
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TSegment3D.h>
#include <mrpt/viz/CArrow.h>
#include <mrpt/viz/CAssimpModel.h>
#include <mrpt/viz/CAxis.h>
#include <mrpt/viz/CBox.h>
#include <mrpt/viz/CCamera.h>
#include <mrpt/viz/CCylinder.h>
#include <mrpt/viz/CEllipsoid2D.h>
#include <mrpt/viz/CEllipsoid3D.h>
#include <mrpt/viz/CFrustum.h>
#include <mrpt/viz/CGridPlaneXY.h>
#include <mrpt/viz/CGridPlaneXZ.h>
#include <mrpt/viz/CPointCloud.h>
#include <mrpt/viz/CPointCloudColoured.h>
#include <mrpt/viz/CSetOfLines.h>
#include <mrpt/viz/CSetOfObjects.h>
#include <mrpt/viz/CSimpleLine.h>
#include <mrpt/viz/CSphere.h>
#include <mrpt/viz/CText.h>
#include <mrpt/viz/CText3D.h>
#include <mrpt/viz/CVisualObject.h>
#include <mrpt/viz/Scene.h>
#include <mrpt/viz/Viewport.h>
#include <mrpt/viz/stock_objects.h>

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
          "getViewport", py::overload_cast<const std::string&>(&Scene::getViewport),
          "name"_a = "main", py::return_value_policy::reference_internal)
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
            if (pts.ndim() != 2 || pts.shape(1) < 3)
            {
              throw std::runtime_error("Expected Nx3 float array");
            }

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

  // =========================================================================
  // Phase 0.4 Extensions — additional viz classes
  // =========================================================================

  // 8. CGridPlaneXY
  py::class_<CGridPlaneXY, CVisualObject, std::shared_ptr<CGridPlaneXY>>(m, "CGridPlaneXY")
      .def(
          py::init<float, float, float, float, float, float>(), py::arg("xmin") = -10.f,
          py::arg("xmax") = 10.f, py::arg("ymin") = -10.f, py::arg("ymax") = 10.f,
          py::arg("z") = 0.f, py::arg("frequency") = 1.f)
      .def(
          "setPlaneLimits", &CGridPlaneXY::setPlaneLimits, py::arg("xmin"), py::arg("xmax"),
          py::arg("ymin"), py::arg("ymax"))
      .def("setPlaneZcoord", &CGridPlaneXY::setPlaneZcoord)
      .def("setGridFrequency", &CGridPlaneXY::setGridFrequency);

  // 9. CGridPlaneXZ
  py::class_<CGridPlaneXZ, CVisualObject, std::shared_ptr<CGridPlaneXZ>>(m, "CGridPlaneXZ")
      .def(
          py::init<float, float, float, float, float, float>(), py::arg("xmin") = -10.f,
          py::arg("xmax") = 10.f, py::arg("zmin") = -10.f, py::arg("zmax") = 10.f,
          py::arg("y") = 0.f, py::arg("frequency") = 1.f)
      .def(
          "setPlaneLimits", &CGridPlaneXZ::setPlaneLimits, py::arg("xmin"), py::arg("xmax"),
          py::arg("zmin"), py::arg("zmax"))
      .def("setPlaneYcoord", &CGridPlaneXZ::setPlaneYcoord)
      .def("setGridFrequency", &CGridPlaneXZ::setGridFrequency);

  // 10. CAxis
  py::class_<CAxis, CVisualObject, std::shared_ptr<CAxis>>(m, "CAxis")
      .def(
          py::init<float, float, float, float, float, float, float, float, bool>(),
          py::arg("xmin") = -1.f, py::arg("ymin") = -1.f, py::arg("zmin") = -1.f,
          py::arg("xmax") = 1.f, py::arg("ymax") = 1.f, py::arg("zmax") = 1.f,
          py::arg("frequency") = 1.f, py::arg("lineWidth") = 3.f, py::arg("marks") = true)
      .def("setAxisLimits", &CAxis::setAxisLimits)
      .def("setFrequency", &CAxis::setFrequency)
      .def("getFrequency", &CAxis::getFrequency)
      .def("setTextScale", &CAxis::setTextScale)
      .def("getTextScale", &CAxis::getTextScale)
      .def("enableTickMarks", py::overload_cast<bool>(&CAxis::enableTickMarks));

  // 11. CBox
  py::class_<CBox, CVisualObject, std::shared_ptr<CBox>>(m, "CBox")
      .def(py::init<>())
      .def(
          py::init<const mrpt::math::TPoint3D&, const mrpt::math::TPoint3D&, bool, float>(),
          py::arg("corner1"), py::arg("corner2"), py::arg("is_wireframe") = false,
          py::arg("lineWidth") = 1.0f)
      .def("setBoxCorners", &CBox::setBoxCorners)
      .def(
          "getBoxCorners",
          [](const CBox& self)
          {
            mrpt::math::TPoint3D c1;
            mrpt::math::TPoint3D c2;
            self.getBoxCorners(c1, c2);
            return py::make_tuple(c1, c2);
          })
      .def("setWireframe", &CBox::setWireframe)
      .def("isWireframe", &CBox::isWireframe)
      .def("enableBoxBorder", &CBox::enableBoxBorder);

  // 12. CSphere
  py::class_<CSphere, CVisualObject, std::shared_ptr<CSphere>>(m, "CSphere")
      .def(py::init<float, int>(), py::arg("radius") = 1.0f, py::arg("nDivs") = 20)
      .def("setRadius", &CSphere::setRadius)
      .def("getRadius", &CSphere::getRadius)
      .def("setNumberDivs", &CSphere::setNumberDivs);

  // 13. CCylinder
  py::class_<CCylinder, CVisualObject, std::shared_ptr<CCylinder>>(m, "CCylinder")
      .def(py::init<>())
      .def(
          py::init<float, float, float, int>(), py::arg("baseRadius"), py::arg("topRadius"),
          py::arg("height") = 1.0f, py::arg("slices") = 20)
      .def("setRadius", &CCylinder::setRadius)
      .def("setRadii", &CCylinder::setRadii)
      .def("setHeight", &CCylinder::setHeight)
      .def("getHeight", &CCylinder::getHeight);

  // 14. CArrow
  py::class_<CArrow, CVisualObject, std::shared_ptr<CArrow>>(m, "CArrow")
      .def(py::init<>())
      .def(
          "setArrowEnds",
          [](CArrow& self, float x0, float y0, float z0, float x1, float y1, float z1)
          { self.setArrowEnds(x0, y0, z0, x1, y1, z1); },
          py::arg("x0"), py::arg("y0"), py::arg("z0"), py::arg("x1"), py::arg("y1"), py::arg("z1"))
      .def("setHeadRatio", &CArrow::setHeadRatio)
      .def("setSmallRadius", &CArrow::setSmallRadius);

  // 15. CText
  py::class_<CText, CVisualObject, std::shared_ptr<CText>>(m, "CText")
      .def(py::init<>())
      .def(py::init<const std::string&>(), py::arg("text"))
      .def("setString", &CText::setString)
      .def("getString", &CText::getString)
      .def("setFont", &CText::setFont);

  // 16. CText3D
  py::class_<CText3D, CVisualObject, std::shared_ptr<CText3D>>(m, "CText3D")
      .def(
          py::init<const std::string&, const std::string&, float>(), py::arg("text") = "",
          py::arg("fontName") = "sans", py::arg("scale") = 1.0f)
      .def("setString", &CText3D::setString)
      .def("getString", &CText3D::getString);

  // 17. CSetOfLines
  py::class_<CSetOfLines, CVisualObject, std::shared_ptr<CSetOfLines>>(m, "CSetOfLines")
      .def(py::init<>())
      .def("clear", &CSetOfLines::clear)
      .def(
          "appendLine",
          static_cast<void (CSetOfLines::*)(double, double, double, double, double, double)>(
              &CSetOfLines::appendLine),
          py::arg("x0"), py::arg("y0"), py::arg("z0"), py::arg("x1"), py::arg("y1"), py::arg("z1"))
      .def(
          "appendLine", static_cast<void (CSetOfLines::*)(const mrpt::math::TSegment3D&)>(
                            &CSetOfLines::appendLine))
      .def("__len__", [](const CSetOfLines& self) { return self.size(); });

  // 18. CSimpleLine
  py::class_<CSimpleLine, CVisualObject, std::shared_ptr<CSimpleLine>>(m, "CSimpleLine")
      .def(py::init<>())
      .def(
          "setLineCoords",
          py::overload_cast<float, float, float, float, float, float>(&CSimpleLine::setLineCoords),
          py::arg("x0"), py::arg("y0"), py::arg("z0"), py::arg("x1"), py::arg("y1"), py::arg("z1"))
      .def("getLineStart", &CSimpleLine::getLineStart)
      .def("getLineEnd", &CSimpleLine::getLineEnd);

  // 19. CEllipsoid3D
  py::class_<CEllipsoid3D, CVisualObject, std::shared_ptr<CEllipsoid3D>>(m, "CEllipsoid3D")
      .def(py::init<>())
      .def(
          "setCovMatrix", [](CEllipsoid3D& self, const mrpt::math::CMatrixDouble33& cov)
          { self.setCovMatrix(cov); })
      .def("setQuantiles", &CEllipsoid3D::setQuantiles)
      .def("set3DsegmentsCount", &CEllipsoid3D::set3DsegmentsCount);

  // 20. CEllipsoid2D
  py::class_<CEllipsoid2D, CVisualObject, std::shared_ptr<CEllipsoid2D>>(m, "CEllipsoid2D")
      .def(py::init<>())
      .def(
          "setCovMatrix", [](CEllipsoid2D& self, const mrpt::math::CMatrixDouble22& cov)
          { self.setCovMatrix(cov); })
      .def("setQuantiles", &CEllipsoid2D::setQuantiles);

  // 21. CPointCloudColoured (complement to existing CPointCloud)
  py::class_<CPointCloudColoured, CVisualObject, std::shared_ptr<CPointCloudColoured>>(
      m, "CPointCloudColoured")
      .def(py::init<>())
      .def("clear", &CPointCloudColoured::clear)
      .def(
          "push_back",
          [](CPointCloudColoured& self, float x, float y, float z, float r, float g, float b,
             float a) { self.push_back(x, y, z, r, g, b, a); },
          py::arg("x"), py::arg("y"), py::arg("z"), py::arg("r") = 1.0f, py::arg("g") = 1.0f,
          py::arg("b") = 1.0f, py::arg("a") = 1.0f)
      .def("size", &CPointCloudColoured::size)
      .def("__len__", [](const CPointCloudColoured& self) { return self.size(); });

  // 22. stock_objects submodule
  auto stock = m.def_submodule("stock_objects", "Pre-built 3D objects");
  stock.def("CornerXYZ", &mrpt::viz::stock_objects::CornerXYZ, py::arg("scale") = 1.0f);
  stock.def(
      "CornerXYZSimple", &mrpt::viz::stock_objects::CornerXYZSimple, py::arg("scale") = 1.0f,
      py::arg("lineWidth") = 1.0f);
  stock.def("CornerXYZEye", &mrpt::viz::stock_objects::CornerXYZEye);
  stock.def(
      "CornerXYSimple", &mrpt::viz::stock_objects::CornerXYSimple, py::arg("scale") = 1.0f,
      py::arg("lineWidth") = 1.0f);
  stock.def("RobotPioneer", &mrpt::viz::stock_objects::RobotPioneer);
  stock.def("RobotRhodon", &mrpt::viz::stock_objects::RobotRhodon);
  stock.def("RobotGiraff", &mrpt::viz::stock_objects::RobotGiraff);
  stock.def("BumblebeeCamera", &mrpt::viz::stock_objects::BumblebeeCamera);
  stock.def("Hokuyo_URG", &mrpt::viz::stock_objects::Hokuyo_URG);
  stock.def("Hokuyo_UTM", &mrpt::viz::stock_objects::Hokuyo_UTM);
}