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
#include <mrpt/viz/CAnimatedAssimpModel.h>
#include <mrpt/viz/CArrow.h>
#include <mrpt/viz/CAssimpModel.h>
#include <mrpt/viz/CAxis.h>
#include <mrpt/viz/CBox.h>
#include <mrpt/viz/CCamera.h>
#include <mrpt/viz/CColorBar.h>
#include <mrpt/viz/CCylinder.h>
#include <mrpt/viz/CDisk.h>
#include <mrpt/viz/CEllipsoid2D.h>
#include <mrpt/viz/CEllipsoid3D.h>
#include <mrpt/viz/CEllipsoidInverseDepth2D.h>
#include <mrpt/viz/CEllipsoidInverseDepth3D.h>
#include <mrpt/viz/CEllipsoidRangeBearing2D.h>
#include <mrpt/viz/CFrustum.h>
#include <mrpt/viz/CGridPlaneXY.h>
#include <mrpt/viz/CGridPlaneXZ.h>
#include <mrpt/viz/CMesh.h>
#include <mrpt/viz/CMesh3D.h>
#include <mrpt/viz/CMeshFast.h>
#include <mrpt/viz/COctoMapVoxels.h>
#include <mrpt/viz/COrbitCameraController.h>
#include <mrpt/viz/CPointCloud.h>
#include <mrpt/viz/CPointCloudColoured.h>
#include <mrpt/viz/CPolyhedron.h>
#include <mrpt/viz/CSetOfLines.h>
#include <mrpt/viz/CSetOfObjects.h>
#include <mrpt/viz/CSetOfTexturedTriangles.h>
#include <mrpt/viz/CSetOfTriangles.h>
#include <mrpt/viz/CSimpleLine.h>
#include <mrpt/viz/CSkyBox.h>
#include <mrpt/viz/CSphere.h>
#include <mrpt/viz/CText.h>
#include <mrpt/viz/CText3D.h>
#include <mrpt/viz/CTexturedPlane.h>
#include <mrpt/viz/CUBE_TEXTURE_FACE.h>
#include <mrpt/viz/CVectorField2D.h>
#include <mrpt/viz/CVectorField3D.h>
#include <mrpt/viz/CVisualObject.h>
#include <mrpt/viz/Scene.h>
#include <mrpt/viz/TLightParameters.h>
#include <mrpt/viz/TTriangle.h>
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
  // TTriangle
  py::class_<TTriangle::Vertex>(m, "TTriangleVertex")
      .def(py::init<>())
      .def(
          "setColor", &TTriangle::Vertex::setColor, py::arg("color"),
          "Set vertex color from TColor");

  py::class_<TTriangle>(m, "TTriangle")
      .def(py::init<>())
      .def(
          py::init<
              const mrpt::math::TPoint3Df&, const mrpt::math::TPoint3Df&,
              const mrpt::math::TPoint3Df&>(),
          py::arg("p1"), py::arg("p2"), py::arg("p3"))
      .def("computeNormals", &TTriangle::computeNormals)
      .def_readwrite("vertices", &TTriangle::vertices);

  // CDisk
  py::class_<CDisk, CVisualObject, std::shared_ptr<CDisk>>(m, "CDisk")
      .def(py::init<>())
      .def(
          py::init<float, float, uint32_t>(), py::arg("out_radius"), py::arg("in_radius"),
          py::arg("slices") = 50U)
      .def(
          "setDiskRadius", &CDisk::setDiskRadius, py::arg("out_radius"),
          py::arg("in_radius") = 0.0f)
      .def_property_readonly("in_radius", &CDisk::getInRadius)
      .def_property_readonly("out_radius", &CDisk::getOutRadius)
      .def("setSlicesCount", &CDisk::setSlicesCount, py::arg("N"));

  // CFrustum
  py::class_<CFrustum, CVisualObject, std::shared_ptr<CFrustum>>(m, "CFrustum")
      .def(py::init<>())
      .def(
          py::init<float, float, float, float, float, bool, bool>(), py::arg("near_distance"),
          py::arg("far_distance"), py::arg("horz_FOV_degrees"), py::arg("vert_FOV_degrees"),
          py::arg("lineWidth") = 1.0f, py::arg("draw_lines") = true, py::arg("draw_planes") = false)
      .def("setNearFarPlanes", &CFrustum::setNearFarPlanes, py::arg("near"), py::arg("far"))
      .def("setHorzFOV", &CFrustum::setHorzFOV, py::arg("fov_degrees"))
      .def("setVertFOV", &CFrustum::setVertFOV, py::arg("fov_degrees"))
      .def_property_readonly("near_plane", &CFrustum::getNearPlaneDistance)
      .def_property_readonly("far_plane", &CFrustum::getFarPlaneDistance)
      .def_property_readonly("horz_fov", &CFrustum::getHorzFOV)
      .def_property_readonly("vert_fov", &CFrustum::getVertFOV)
      .def("setPlaneColor", &CFrustum::setPlaneColor, py::arg("color"));

  // CSetOfTriangles
  py::class_<CSetOfTriangles, CVisualObject, std::shared_ptr<CSetOfTriangles>>(m, "CSetOfTriangles")
      .def(py::init<>())
      .def("clearTriangles", &CSetOfTriangles::clearTriangles)
      .def("getTrianglesCount", &CSetOfTriangles::getTrianglesCount)
      .def(
          "getTriangle",
          [](const CSetOfTriangles& self, size_t idx)
          {
            TTriangle t;
            self.getTriangle(idx, t);
            return t;
          },
          py::arg("idx"))
      .def("insertTriangle", &CSetOfTriangles::insertTriangle, py::arg("triangle"));

  // CVectorField2D
  py::class_<CVectorField2D, CVisualObject, std::shared_ptr<CVectorField2D>>(m, "CVectorField2D")
      .def(py::init<>())
      .def("clear", &CVectorField2D::clear)
      .def(
          "setGridLimits", &CVectorField2D::setGridLimits, py::arg("xmin"), py::arg("xmax"),
          py::arg("ymin"), py::arg("ymax"))
      .def(
          "setGridCenterAndCellSize", &CVectorField2D::setGridCenterAndCellSize, py::arg("cx"),
          py::arg("cy"), py::arg("cell_x"), py::arg("cell_y"))
      .def(
          "setPointColor", &CVectorField2D::setPointColor, py::arg("R"), py::arg("G"), py::arg("B"),
          py::arg("A") = 1.0f)
      .def(
          "setVectorFieldColor", &CVectorField2D::setVectorFieldColor, py::arg("R"), py::arg("G"),
          py::arg("B"), py::arg("A") = 1.0f)
      .def(
          "setVectorField",
          [](CVectorField2D& self, mrpt::math::CMatrixFloat vx, mrpt::math::CMatrixFloat vy)
          { self.setVectorField(vx, vy); },
          py::arg("vx"), py::arg("vy"));

  // CVectorField3D
  py::class_<CVectorField3D, CVisualObject, std::shared_ptr<CVectorField3D>>(m, "CVectorField3D")
      .def(py::init<>())
      .def("clear", &CVectorField3D::clear)
      .def(
          "setPointColor", &CVectorField3D::setPointColor, py::arg("R"), py::arg("G"), py::arg("B"),
          py::arg("A") = 1.0f)
      .def(
          "setVectorFieldColor", &CVectorField3D::setVectorFieldColor, py::arg("R"), py::arg("G"),
          py::arg("B"), py::arg("A") = 1.0f)
      .def("setMaxSpeedForColor", &CVectorField3D::setMaxSpeedForColor, py::arg("s"))
      .def(
          "setVectorField",
          [](CVectorField3D& self, mrpt::math::CMatrixFloat vx, mrpt::math::CMatrixFloat vy,
             mrpt::math::CMatrixFloat vz) { self.setVectorField(vx, vy, vz); },
          py::arg("vx"), py::arg("vy"), py::arg("vz"))
      .def(
          "setPointCoordinates",
          [](CVectorField3D& self, mrpt::math::CMatrixFloat px, mrpt::math::CMatrixFloat py2,
             mrpt::math::CMatrixFloat pz) { self.setPointCoordinates(px, py2, pz); },
          py::arg("px"), py::arg("py"), py::arg("pz"));

  // CMesh
  py::class_<CMesh, CVisualObject, std::shared_ptr<CMesh>>(m, "CMesh")
      .def(
          py::init<bool, float, float, float, float>(), py::arg("enable_transparency") = false,
          py::arg("xMin") = -1.0f, py::arg("xMax") = 1.0f, py::arg("yMin") = -1.0f,
          py::arg("yMax") = 1.0f)
      .def(
          "setGridLimits",
          [](CMesh& self, float x0, float x1, float y0, float y1)
          { self.setGridLimits(x0, x1, y0, y1); },
          py::arg("xMin"), py::arg("xMax"), py::arg("yMin"), py::arg("yMax"))
      .def("enableTransparency", &CMesh::enableTransparency, py::arg("v"))
      .def("enableWireFrame", &CMesh::enableWireFrame, py::arg("v"))
      .def(
          "enableColorFromZ",
          [](CMesh& self, bool v) { self.enableColorFromZ(v, mrpt::img::cmHOT); }, py::arg("v"),
          "Enable color from Z height using HOT colormap")
      .def(
          "setZ",
          [](CMesh& self, const py::array_t<float>& arr)
          {
            auto buf = arr.request();
            const auto rows = static_cast<int>(buf.shape[0]);
            const auto cols = static_cast<int>(buf.shape[1]);
            mrpt::math::CMatrixDynamic<float> Z(rows, cols);
            const auto* src = static_cast<const float*>(buf.ptr);
            for (int r = 0; r < rows; r++)
            {
              for (int c = 0; c < cols; c++)
              {
                Z(r, c) = src[r * cols + c];
              }
            }
            self.setZ(Z);
          },
          py::arg("Z"), "Set height matrix (numpy float32 2D array)");

  // =========================================================================
  // Phase 0.5 Extensions — additional viz classes (item #3)
  // =========================================================================

  // CColorBar (TColormap is an int enum in C++)
  py::class_<CColorBar, CVisualObject, std::shared_ptr<CColorBar>>(m, "CColorBar")
      .def(
          py::init(
              [](int colormap, double width, double height, float min_col, float max_col,
                 float min_value, float max_value, const std::string& label_format,
                 float label_font_size)
              {
                return std::make_shared<CColorBar>(
                    static_cast<mrpt::img::TColormap>(colormap), width, height, min_col, max_col,
                    min_value, max_value, label_format, label_font_size);
              }),
          py::arg("colormap") = 0, py::arg("width") = 0.2, py::arg("height") = 1.0,
          py::arg("min_col") = 0.0f, py::arg("max_col") = 1.0f, py::arg("min_value") = 0.0f,
          py::arg("max_value") = 1.0f, py::arg("label_format") = std::string("%7.02f"),
          py::arg("label_font_size") = 0.05f)
      .def(
          "setColormap",
          [](CColorBar& self, int colormap)
          { self.setColormap(static_cast<mrpt::img::TColormap>(colormap)); },
          py::arg("colormap"))
      .def(
          "setColorAndValueLimits", &CColorBar::setColorAndValueLimits, py::arg("col_min"),
          py::arg("col_max"), py::arg("value_min"), py::arg("value_max"));

  // CMesh3D
  py::class_<CMesh3D, CVisualObject, std::shared_ptr<CMesh3D>>(m, "CMesh3D")
      .def(py::init<>())
      .def("enableShowEdges", &CMesh3D::enableShowEdges, py::arg("v"))
      .def("enableShowFaces", &CMesh3D::enableShowFaces, py::arg("v"))
      .def("enableShowVertices", &CMesh3D::enableShowVertices, py::arg("v"))
      .def("enableFaceNormals", &CMesh3D::enableFaceNormals, py::arg("v"))
      .def(
          "loadMesh",
          [](CMesh3D& self, const py::array_t<float>& verts, const py::array_t<int>& face_verts,
             const py::array_t<int>& verts_per_face)
          {
            auto v = verts.unchecked<2>();
            auto fv = face_verts.unchecked<1>();
            auto vpf = verts_per_face.unchecked<1>();
            const auto nv = static_cast<unsigned int>(v.shape(0));
            const auto nf = static_cast<unsigned int>(vpf.shape(0));
            std::vector<float> vc(static_cast<size_t>(nv) * 3);
            for (unsigned int i = 0; i < nv; i++)
            {
              vc[static_cast<size_t>(i) * 3] = v(i, 0);
              vc[static_cast<size_t>(i) * 3 + 1] = v(i, 1);
              vc[static_cast<size_t>(i) * 3 + 2] = v(i, 2);
            }
            std::vector<int> vpf_v(nf);
            std::vector<int> fv_v(fv.shape(0));
            for (unsigned int i = 0; i < nf; i++)
            {
              vpf_v[i] = vpf(i);
            }
            for (ssize_t i = 0; i < fv.shape(0); i++)
            {
              fv_v[i] = fv(i);
            }
            self.loadMesh(nv, nf, vpf_v.data(), fv_v.data(), vc.data());
          },
          py::arg("vertices"), py::arg("face_vertices"), py::arg("verts_per_face"),
          "Load a mesh from numpy arrays: vertices (Nx3 float32), face_vertices (M,) int32, "
          "verts_per_face (F,) int32");

  // CMeshFast
  py::class_<CMeshFast, CVisualObject, std::shared_ptr<CMeshFast>>(m, "CMeshFast")
      .def(
          py::init<bool, float, float, float, float>(), py::arg("enable_transparency") = false,
          py::arg("xMin") = -1.0f, py::arg("xMax") = 1.0f, py::arg("yMin") = -1.0f,
          py::arg("yMax") = 1.0f)
      .def(
          "setGridLimits", &CMeshFast::setGridLimits, py::arg("xmin"), py::arg("xmax"),
          py::arg("ymin"), py::arg("ymax"))
      .def("enableTransparency", &CMeshFast::enableTransparency, py::arg("v"))
      .def(
          "enableColorFromZ",
          [](CMeshFast& self, bool v, int colormap)
          { self.enableColorFromZ(v, static_cast<mrpt::img::TColormap>(colormap)); },
          py::arg("v"), py::arg("colormap") = 4, "Enable color from Z height (colormap int)")
      .def(
          "setZ",
          [](CMeshFast& self, const py::array_t<float>& arr)
          {
            auto buf = arr.request();
            const auto rows = static_cast<int>(buf.shape[0]);
            const auto cols = static_cast<int>(buf.shape[1]);
            mrpt::math::CMatrixDynamic<float> Z(rows, cols);
            const auto* src = static_cast<const float*>(buf.ptr);
            for (int r = 0; r < rows; r++)
            {
              for (int c = 0; c < cols; c++)
              {
                Z(r, c) = src[r * cols + c];
              }
            }
            self.setZ(Z);
          },
          py::arg("Z"), "Set height matrix (numpy float32 2D array)");

  // CTexturedPlane
  py::class_<CTexturedPlane, CVisualObject, std::shared_ptr<CTexturedPlane>>(m, "CTexturedPlane")
      .def(
          py::init<float, float, float, float>(), py::arg("x_min") = -1.0f, py::arg("x_max") = 1.0f,
          py::arg("y_min") = -1.0f, py::arg("y_max") = 1.0f)
      .def(
          "setPlaneCorners", &CTexturedPlane::setPlaneCorners, py::arg("xMin"), py::arg("xMax"),
          py::arg("yMin"), py::arg("yMax"))
      .def(
          "getPlaneCorners",
          [](const CTexturedPlane& self)
          {
            float xMin = 0.0f;
            float xMax = 0.0f;
            float yMin = 0.0f;
            float yMax = 0.0f;
            self.getPlaneCorners(xMin, xMax, yMin, yMax);
            return py::make_tuple(xMin, xMax, yMin, yMax);
          })
      .def(
          "setTextureRepeat", &CTexturedPlane::setTextureRepeat, py::arg("repeatX"),
          py::arg("repeatY"))
      .def("enableLighting", &CTexturedPlane::enableLighting, py::arg("enable") = true);

  // CSetOfTexturedTriangles
  py::class_<CSetOfTexturedTriangles, CVisualObject, std::shared_ptr<CSetOfTexturedTriangles>>(
      m, "CSetOfTexturedTriangles")
      .def(py::init([]() { return std::make_shared<CSetOfTexturedTriangles>(); }))
      .def("clearTriangles", [](CSetOfTexturedTriangles& self) { self.clearTriangles(); })
      .def(
          "getTrianglesCount",
          [](const CSetOfTexturedTriangles& self) { return self.getTrianglesCount(); })
      .def(
          "getTriangle",
          [](const CSetOfTexturedTriangles& self, size_t idx) -> mrpt::viz::TTriangle
          { return self.getTriangle(idx); },
          py::arg("idx"))
      .def(
          "insertTriangle",
          [](CSetOfTexturedTriangles& self, const mrpt::viz::TTriangle& t)
          { self.insertTriangle(t); },
          py::arg("triangle"));

  // CPolyhedron
  py::class_<CPolyhedron, CVisualObject, std::shared_ptr<CPolyhedron>>(m, "CPolyhedron")
      .def_static("CreateTetrahedron", &CPolyhedron::CreateTetrahedron, py::arg("radius"))
      .def_static("CreateHexahedron", &CPolyhedron::CreateHexahedron, py::arg("radius"))
      .def_static("CreateOctahedron", &CPolyhedron::CreateOctahedron, py::arg("radius"))
      .def_static("CreateDodecahedron", &CPolyhedron::CreateDodecahedron, py::arg("radius"))
      .def_static("CreateIcosahedron", &CPolyhedron::CreateIcosahedron, py::arg("radius"))
      .def_static(
          "CreateTruncatedTetrahedron", &CPolyhedron::CreateTruncatedTetrahedron, py::arg("radius"))
      .def_static(
          "CreateTruncatedHexahedron", &CPolyhedron::CreateTruncatedHexahedron, py::arg("radius"))
      .def_static(
          "CreateTruncatedOctahedron", &CPolyhedron::CreateTruncatedOctahedron, py::arg("radius"))
      .def_static(
          "CreateTruncatedIcosahedron", &CPolyhedron::CreateTruncatedIcosahedron, py::arg("radius"))
      .def_static("CreateCuboctahedron", &CPolyhedron::CreateCuboctahedron, py::arg("radius"))
      .def_static(
          "CreateIcosidodecahedron",
          py::overload_cast<double, bool>(&CPolyhedron::CreateIcosidodecahedron), py::arg("radius"),
          py::arg("type") = true);

  // COrbitCameraController
  py::class_<COrbitCameraController>(m, "COrbitCameraController")
      .def(py::init<>())
      .def(
          "setCameraPointing",
          py::overload_cast<float, float, float>(&COrbitCameraController::setCameraPointing),
          py::arg("x"), py::arg("y"), py::arg("z"))
      .def_property(
          "zoom", [](const COrbitCameraController& c) { return c.getZoomDistance(); },
          [](COrbitCameraController& c, float d) { c.setZoomDistance(d); })
      .def_property(
          "azimuth_deg", [](const COrbitCameraController& c) { return c.getAzimuthDegrees(); },
          [](COrbitCameraController& c, float d) { c.setAzimuthDegrees(d); })
      .def_property(
          "elevation_deg", [](const COrbitCameraController& c) { return c.getElevationDegrees(); },
          [](COrbitCameraController& c, float d) { c.setElevationDegrees(d); })
      .def_property(
          "roll_deg", [](const COrbitCameraController& c) { return c.getRollDegrees(); },
          [](COrbitCameraController& c, float d) { c.setRollDegrees(d); })
      .def("setZoomDistance", &COrbitCameraController::setZoomDistance, py::arg("d"))
      .def("setAzimuthDegrees", &COrbitCameraController::setAzimuthDegrees, py::arg("deg"))
      .def("setElevationDegrees", &COrbitCameraController::setElevationDegrees, py::arg("deg"))
      .def("applyTo", &COrbitCameraController::applyTo, py::arg("cam"))
      .def("setFrom", &COrbitCameraController::setFrom, py::arg("cam"))
      .def(
          "onMouseMove", &COrbitCameraController::onMouseMove, py::arg("x"), py::arg("y"),
          py::arg("buttons"), py::arg("modifiers"))
      .def(
          "onMouseButton", &COrbitCameraController::onMouseButton, py::arg("x"), py::arg("y"),
          py::arg("button"), py::arg("down"))
      .def("onScroll", &COrbitCameraController::onScroll, py::arg("delta"), py::arg("modifiers"));

  // COctoMapVoxels
  py::enum_<COctoMapVoxels::visualization_mode_t>(m, "OctoMapVisualizationMode")
      .value(
          "FIXED", COctoMapVoxels::visualization_mode_t::FIXED,
          "All voxels have the same fixed color")
      .value(
          "COLOR_FROM_HEIGHT", COctoMapVoxels::visualization_mode_t::COLOR_FROM_HEIGHT,
          "Color voxels by height")
      .value(
          "COLOR_FROM_OCCUPANCY", COctoMapVoxels::visualization_mode_t::COLOR_FROM_OCCUPANCY,
          "Color by occupancy probability")
      .value(
          "TRANSPARENCY_FROM_OCCUPANCY",
          COctoMapVoxels::visualization_mode_t::TRANSPARENCY_FROM_OCCUPANCY,
          "Transparency from occupancy")
      .value(
          "TRANS_AND_COLOR_FROM_OCCUPANCY",
          COctoMapVoxels::visualization_mode_t::TRANS_AND_COLOR_FROM_OCCUPANCY,
          "Both transparency and color from occupancy")
      .value(
          "COLOR_FROM_RGB_DATA", COctoMapVoxels::visualization_mode_t::COLOR_FROM_RGB_DATA,
          "Use per-voxel stored RGB")
      .export_values();

  py::class_<COctoMapVoxels, CVisualObject, std::shared_ptr<COctoMapVoxels>>(m, "COctoMapVoxels")
      .def(py::init<>())
      .def("clear", &COctoMapVoxels::clear)
      .def("setVisualizationMode", &COctoMapVoxels::setVisualizationMode, py::arg("mode"))
      .def("enableLights", &COctoMapVoxels::enableLights, py::arg("enable"))
      .def("enableCubeTransparency", &COctoMapVoxels::enableCubeTransparency, py::arg("enable"))
      .def("showGridLines", &COctoMapVoxels::showGridLines, py::arg("show"))
      .def("showVoxels", &COctoMapVoxels::showVoxels, py::arg("voxel_set"), py::arg("show"))
      .def("showVoxelsAsPoints", &COctoMapVoxels::showVoxelsAsPoints, py::arg("enable"))
      .def("getVoxelCount", &COctoMapVoxels::getVoxelCount, py::arg("set_index"))
      .def("getVoxelSetCount", &COctoMapVoxels::getVoxelSetCount)
      .def("resizeVoxelSets", &COctoMapVoxels::resizeVoxelSets, py::arg("n"))
      .def("resizeVoxels", &COctoMapVoxels::resizeVoxels, py::arg("set_index"), py::arg("n"))
      .def(
          "push_back_Voxel",
          [](COctoMapVoxels& self, size_t set_idx, float x, float y, float z, float side, uint8_t r,
             uint8_t g, uint8_t b, uint8_t a)
          {
            self.push_back_Voxel(
                set_idx, COctoMapVoxels::TVoxel(
                             mrpt::math::TPoint3Df(x, y, z), side, mrpt::img::TColor(r, g, b, a)));
          },
          py::arg("set_index"), py::arg("x"), py::arg("y"), py::arg("z"), py::arg("side"),
          py::arg("r") = 200, py::arg("g") = 200, py::arg("b") = 200, py::arg("a") = 255);

  // CUBE_TEXTURE_FACE enum
  py::enum_<CUBE_TEXTURE_FACE>(m, "CubeTextureFace")
      .value("LEFT", CUBE_TEXTURE_FACE::LEFT)
      .value("RIGHT", CUBE_TEXTURE_FACE::RIGHT)
      .value("TOP", CUBE_TEXTURE_FACE::TOP)
      .value("BOTTOM", CUBE_TEXTURE_FACE::BOTTOM)
      .value("FRONT", CUBE_TEXTURE_FACE::FRONT)
      .value("BACK", CUBE_TEXTURE_FACE::BACK)
      .export_values();

  // CSkyBox
  py::class_<CSkyBox, CVisualObject, std::shared_ptr<CSkyBox>>(m, "CSkyBox")
      .def(py::init<>())
      .def(
          "assignImage",
          py::overload_cast<const CUBE_TEXTURE_FACE, const mrpt::img::CImage&>(
              &CSkyBox::assignImage),
          py::arg("face"), py::arg("img"));

  // TLightType enum
  py::enum_<TLightType>(m, "TLightType")
      .value("Directional", TLightType::Directional)
      .value("Point", TLightType::Point)
      .value("Spot", TLightType::Spot)
      .export_values();

  // TLight struct
  py::class_<TLight>(m, "TLight")
      .def(py::init<>())
      .def_readwrite("type", &TLight::type)
      .def_readwrite("diffuse", &TLight::diffuse)
      .def_readwrite("specular", &TLight::specular)
      .def_readwrite("direction", &TLight::direction)
      .def_readwrite("position", &TLight::position)
      .def_readwrite("attenuation_constant", &TLight::attenuation_constant)
      .def_readwrite("attenuation_linear", &TLight::attenuation_linear)
      .def_readwrite("attenuation_quadratic", &TLight::attenuation_quadratic)
      .def_readwrite("spot_inner_cutoff_deg", &TLight::spot_inner_cutoff_deg)
      .def_readwrite("spot_outer_cutoff_deg", &TLight::spot_outer_cutoff_deg)
      .def_static(
          "Directional",
          [](const mrpt::math::TVector3Df& dir, float r, float g, float b, float diffuse,
             float specular)
          { return TLight::Directional(dir, mrpt::img::TColorf(r, g, b), diffuse, specular); },
          py::arg("dir"), py::arg("r") = 1.0f, py::arg("g") = 1.0f, py::arg("b") = 1.0f,
          py::arg("diffuse") = 0.8f, py::arg("specular") = 0.95f);

  // Generalized ellipsoids: CEllipsoidInverseDepth2D, CEllipsoidInverseDepth3D,
  // CEllipsoidRangeBearing2D. CGeneralizedEllipsoidTemplate<N> has a protected destructor so
  // we cannot register the base; bind the concrete classes directly under CVisualObject instead.
  py::class_<CEllipsoidInverseDepth2D, CVisualObject, std::shared_ptr<CEllipsoidInverseDepth2D>>(
      m, "CEllipsoidInverseDepth2D")
      .def(py::init<>())
      .def("setQuantiles", &CEllipsoidInverseDepth2D::setQuantiles, py::arg("q"))
      .def("getQuantiles", &CEllipsoidInverseDepth2D::getQuantiles)
      .def(
          "setCovMatrix", [](CEllipsoidInverseDepth2D& self, const mrpt::math::CMatrixDouble22& cov)
          { self.setCovMatrix(cov); })
      .def(
          "setUnderflowMaxRange", &CEllipsoidInverseDepth2D::setUnderflowMaxRange,
          py::arg("maxRange"))
      .def("getUnderflowMaxRange", &CEllipsoidInverseDepth2D::getUnderflowMaxRange);

  py::class_<CEllipsoidInverseDepth3D, CVisualObject, std::shared_ptr<CEllipsoidInverseDepth3D>>(
      m, "CEllipsoidInverseDepth3D")
      .def(py::init<>())
      .def("setQuantiles", &CEllipsoidInverseDepth3D::setQuantiles, py::arg("q"))
      .def("getQuantiles", &CEllipsoidInverseDepth3D::getQuantiles)
      .def(
          "setCovMatrix", [](CEllipsoidInverseDepth3D& self, const mrpt::math::CMatrixDouble33& cov)
          { self.setCovMatrix(cov); })
      .def(
          "setUnderflowMaxRange", &CEllipsoidInverseDepth3D::setUnderflowMaxRange,
          py::arg("maxRange"))
      .def("getUnderflowMaxRange", &CEllipsoidInverseDepth3D::getUnderflowMaxRange);

  py::class_<CEllipsoidRangeBearing2D, CVisualObject, std::shared_ptr<CEllipsoidRangeBearing2D>>(
      m, "CEllipsoidRangeBearing2D")
      .def(py::init<>())
      .def("setQuantiles", &CEllipsoidRangeBearing2D::setQuantiles, py::arg("q"))
      .def("getQuantiles", &CEllipsoidRangeBearing2D::getQuantiles)
      .def(
          "setCovMatrix", [](CEllipsoidRangeBearing2D& self, const mrpt::math::CMatrixDouble22& cov)
          { self.setCovMatrix(cov); });

  // CAnimatedAssimpModel
  py::class_<CAnimatedAssimpModel, CAssimpModel, std::shared_ptr<CAnimatedAssimpModel>>(
      m, "CAnimatedAssimpModel")
      .def(py::init<>())
      .def(
          "loadScene",
          [](CAnimatedAssimpModel& self, const std::string& fn, int flags)
          { self.loadScene(fn, flags); },
          py::arg("file_name"),
          py::arg("flags") =
              (CAssimpModel::LoadFlags::RealTimeMaxQuality | CAssimpModel::LoadFlags::FlipUVs |
               CAssimpModel::LoadFlags::Verbose))
      .def("setAnimationTime", &CAnimatedAssimpModel::setAnimationTime, py::arg("t_seconds"))
      .def(
          "getAnimationDuration", &CAnimatedAssimpModel::getAnimationDuration,
          py::arg("anim_idx") = 0)
      .def("getAnimationCount", &CAnimatedAssimpModel::getAnimationCount)
      .def("getAnimationName", &CAnimatedAssimpModel::getAnimationName, py::arg("anim_idx"))
      .def(
          "setActiveAnimation",
          py::overload_cast<const std::string&>(&CAnimatedAssimpModel::setActiveAnimation),
          py::arg("anim_name"))
      .def(
          "setActiveAnimationByIndex",
          py::overload_cast<size_t>(&CAnimatedAssimpModel::setActiveAnimation), py::arg("idx"))
      .def("setLooping", &CAnimatedAssimpModel::setLooping, py::arg("loop"));

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