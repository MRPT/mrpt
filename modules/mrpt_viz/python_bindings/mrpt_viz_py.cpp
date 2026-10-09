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
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

// MRPT headers
#include <mrpt/math/CMatrixF.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TSegment3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPosePDF.h>
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
#include <mrpt/viz/CLight.h>
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

namespace
{
// Float matrices are passed from Python as 2D NumPy arrays:
using FloatMatrix = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

mrpt::math::CMatrixFloat toCMatrixFloat(const FloatMatrix& m)
{
  mrpt::math::CMatrixFloat ret(m.rows(), m.cols());
  ret.asEigen() = m;
  return ret;
}
}  // namespace

PYBIND11_MODULE(_bindings, m)
{
  // 1. Base Class: CVisualObject
  py::class_<CVisualObject, std::shared_ptr<CVisualObject>>(
      m, "CVisualObject", "Base class of all 3D objects that can be rendered.")
      .def_property("name", &CVisualObject::getName, &CVisualObject::setName)
      .def_property(
          "pose", &CVisualObject::getPose,
          static_cast<CVisualObject& (CVisualObject::*)(const mrpt::poses::CPose3D&)>(
              &CVisualObject::setPose),
          "Pose of the object with respect to its parent (read as a TPose3D, set from a CPose3D).")
      .def_property("visible", &CVisualObject::isVisible, &CVisualObject::setVisibility)
      .def(
          "setColor",
          [](CVisualObject& self, uint8_t r, uint8_t g, uint8_t b, uint8_t a)
          { self.setColor(mrpt::img::TColorf(mrpt::img::TColor(r, g, b, a))); },
          "r"_a, "g"_a, "b"_a, "a"_a = 255, "Sets the object color from 8-bit components (0-255).")
      .def(
          "setColor", [](CVisualObject& self, const mrpt::img::TColorf& c) { self.setColor(c); },
          "color"_a, "Sets the color from a TColorf (float components in [0,1])")
      .def(
          "setColor",
          [](CVisualObject& self, const mrpt::img::TColor& c)
          { self.setColor(mrpt::img::TColorf(c)); },
          "color"_a, "Sets the color from a TColor (uint8 components)")
      .def(
          "getColor", [](const CVisualObject& self) { return self.getColor(); },
          "Get color components as floats in the range [0,1].")
      .def(
          "setPose", [](CVisualObject& self, const mrpt::poses::CPose3D& p) { self.setPose(p); },
          "pose"_a, "Sets the pose of the object with respect to its parent.")
      .def(
          "setPose",
          [](CVisualObject& self, const mrpt::poses::CPose2D& p)
          { self.setPose(mrpt::poses::CPose3D(p)); },
          "pose"_a, "Sets the pose of the object with respect to its parent, from a 2D pose.")
      .def("getPose", &CVisualObject::getPose, "Returns the 3D pose of the object as TPose3D.")
      .def(
          "setColor",
          [](CVisualObject& self, float r, float g, float b, float a)
          { self.setColor(r, g, b, a); },
          "r"_a, "g"_a, "b"_a, "a"_a = 1.0f, "Sets the color from float components in [0,1]")
      .def(
          "setLocation",
          [](CVisualObject& self, double x, double y, double z) { self.setLocation(x, y, z); },
          "x"_a, "y"_a, "z"_a, "Changes the position, keeping the orientation")
      .def(
          "setLocation",
          [](CVisualObject& self, const mrpt::math::TPoint3D& p) { self.setLocation(p); }, "p"_a,
          "Changes the location of the object, keeping untouched the orientation.")
      .def(
          "setScale", [](CVisualObject& self, float s) { self.setScale(s); }, "s"_a,
          "Sets the same scale factor in x, y and z")
      .def(
          "setScale",
          [](CVisualObject& self, float sx, float sy, float sz) { self.setScale(sx, sy, sz); },
          "sx"_a, "sy"_a, "sz"_a,
          "Sets the scale factor applied to the object along each axis (default: 1).")
      .def_property(
          "castShadows", [](const CVisualObject& self) { return self.castShadows(); },
          [](CVisualObject& self, bool doCast) { self.castShadows(doCast); },
          "Whether the object casts shadows (if shadows are enabled in the viewport)");

  // Every class deriving from CVisualObject is registered with
  // py::multiple_inheritance(): most renderables inherit CVisualObject
  // virtually (through the VisualObjectParams_* mixins), so it is not at
  // offset zero, and pybind11's single-inheritance fast path would reinterpret
  // the pointer without the virtual-base adjustment.
  // 2. CSetOfObjects (The container node)
  py::class_<CSetOfObjects, CVisualObject, std::shared_ptr<CSetOfObjects>>(
      m, "CSetOfObjects", "A group of 3D objects, placed relative to the pose of this object.",
      py::multiple_inheritance())
      .def(py::init<>(), "Default constructor.")
      .def(
          "insert",
          static_cast<void (CSetOfObjects::*)(const CVisualObject::Ptr&)>(&CSetOfObjects::insert),
          "obj"_a, "Adds an object to the set.")
      .def("clear", &CSetOfObjects::clear, "Removes all objects.")
      .def("__len__", [](const CSetOfObjects& self) { return self.size(); });

  // Registered before Viewport, whose getCamera() returns it:
  py::class_<CCamera, CVisualObject, std::shared_ptr<CCamera>> camera(
      m, "CCamera",
      "Defines the intrinsic and extrinsic camera coordinates from which to render a 3D scene.",
      py::multiple_inheritance());

  // 3. Viewport
  py::class_<Viewport, std::shared_ptr<Viewport>>(
      m, "Viewport", "A viewport within a Scene, containing a set of OpenGL objects to render.")
      .def_property_readonly("name", &Viewport::getName, "Returns the name of the viewport.")
      .def("insert", &Viewport::insert, "obj"_a, "Adds an object to the viewport.")
      .def("clear", &Viewport::clear, "Removes all objects.")
      .def(
          "setViewportPosition", &Viewport::setViewportPosition,
          "Change the viewport position and dimension on the rendering window.")
      .def(
          "setCustomBackgroundColor", &Viewport::setCustomBackgroundColor,
          "Defines the viewport background color.")
      .def(
          "getCamera", static_cast<CCamera& (Viewport::*)()>(&Viewport::getCamera),
          py::return_value_policy::reference_internal, "Returns the camera of this viewport.");

  // 4. Scene (Top-level)
  py::class_<Scene, std::shared_ptr<Scene>>(
      m, "Scene", "A 3D scene: one or more viewports, each with a set of objects to render.")
      .def(py::init<>(), "Default constructor.")
      .def(
          "getViewport", py::overload_cast<const std::string&>(&Scene::getViewport),
          "name"_a = "main", py::return_value_policy::reference_internal,
          "Returns the viewport with the given name (default: \"main\"), or None.")
      .def(
          "createViewport", &Scene::createViewport, "name"_a,
          "Creates a new viewport with the given name, and returns it.")
      .def(
          "clear", &Scene::clear, "createMainViewport"_a = true,
          "Removes all objects (and viewports), then re-creates the main viewport")
      .def(
          "insert",
          [](Scene& self, const CVisualObject::Ptr& obj) { self.getViewport()->insert(obj); },
          "Insert a new object into the scene, in the given viewport (by default, into the "
          "\"main\" viewport).");

  // 5. CCamera
  camera.def(py::init<>(), "Default constructor.")
      .def(
          "setAzimuthDegrees", &CCamera::setAzimuthDegrees,
          "Sets the camera azimuth angle, in degrees.")
      .def(
          "setElevationDegrees", &CCamera::setElevationDegrees,
          "Sets the camera elevation angle, in degrees.")
      .def(
          "setZoomDistance", &CCamera::setZoomDistance,
          "Sets the camera distance to the point it looks at.");

  // 6. CPointCloud
  py::class_<CPointCloud, CVisualObject, std::shared_ptr<CPointCloud>>(
      m, "CPointCloud",
      "A cloud of points, all with the same color or each depending on its value along a "
      "particular coordinate axis.",
      py::multiple_inheritance())
      .def(py::init<>(), "Constructor.")
      .def("clear", &CPointCloud::clear, "Empty the list of points.")
      .def("setPointSize", &CPointCloud::setPointSize, "pointSize"_a, "Point size, in pixels")
      .def(
          "getPointSize", &CPointCloud::getPointSize, "Returns the rendered point size, in pixels.")
      .def(
          "insertPoint",
          static_cast<void (CPointCloud::*)(float, float, float)>(&CPointCloud::insertPoint), "x"_a,
          "y"_a, "z"_a, "Adds a new point to the cloud.")
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
            for (py::ssize_t i = 0; i < r.shape(0); i++)
            {
              self.insertPoint(r(i, 0), r(i, 1), r(i, 2));
            }
          },
          "Sets all points from a NumPy array of shape (N, 3).");

  // 7. CAssimpModel
  py::class_<CAssimpModel, CVisualObject, std::shared_ptr<CAssimpModel>>(
      m, "CAssimpModel", "A 3D model loaded from any file format supported by the Assimp library.",
      py::multiple_inheritance())
      .def(py::init<>(), "Default constructor.")
      .def(
          "loadScene", &CAssimpModel::loadScene, py::arg("file_name"),
          py::arg("flags") =
              (CAssimpModel::LoadFlags::RealTimeMaxQuality | CAssimpModel::LoadFlags::FlipUVs |
               CAssimpModel::LoadFlags::Verbose),
          "Loads a 3D scene from a file in any Assimp-supported format.");

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
  py::class_<CGridPlaneXY, CVisualObject, std::shared_ptr<CGridPlaneXY>>(
      m, "CGridPlaneXY", "A grid of lines over the XY plane.", py::multiple_inheritance())
      .def(
          py::init<float, float, float, float, float, float>(), py::arg("xmin") = -10.f,
          py::arg("xmax") = 10.f, py::arg("ymin") = -10.f, py::arg("ymax") = 10.f,
          py::arg("z") = 0.f, py::arg("frequency") = 1.f,
          "Builds the grid from its limits, height and spacing.")
      .def(
          "setPlaneLimits", &CGridPlaneXY::setPlaneLimits, py::arg("xmin"), py::arg("xmax"),
          py::arg("ymin"), py::arg("ymax"), "Sets the grid limits in x and y.")
      .def("setPlaneZcoord", &CGridPlaneXY::setPlaneZcoord, "Sets the grid height (z).")
      .def(
          "setGridFrequency", &CGridPlaneXY::setGridFrequency,
          "Sets the spacing between grid lines.");

  // 9. CGridPlaneXZ
  py::class_<CGridPlaneXZ, CVisualObject, std::shared_ptr<CGridPlaneXZ>>(
      m, "CGridPlaneXZ", "A grid of lines over the XZ plane.", py::multiple_inheritance())
      .def(
          py::init<float, float, float, float, float, float>(), py::arg("xmin") = -10.f,
          py::arg("xmax") = 10.f, py::arg("zmin") = -10.f, py::arg("zmax") = 10.f,
          py::arg("y") = 0.f, py::arg("frequency") = 1.f,
          "Builds the grid from its limits, y coordinate and spacing.")
      .def(
          "setPlaneLimits", &CGridPlaneXZ::setPlaneLimits, py::arg("xmin"), py::arg("xmax"),
          py::arg("zmin"), py::arg("zmax"), "Sets the grid limits in x and z.")
      .def("setPlaneYcoord", &CGridPlaneXZ::setPlaneYcoord, "Sets the grid y coordinate.")
      .def(
          "setGridFrequency", &CGridPlaneXZ::setGridFrequency,
          "Sets the spacing between grid lines.");

  // 10. CAxis
  py::class_<CAxis, CVisualObject, std::shared_ptr<CAxis>>(
      m, "CAxis", "Draw a 3D world axis, with coordinate marks at some regular interval.",
      py::multiple_inheritance())
      .def(
          py::init<float, float, float, float, float, float, float, float, bool>(),
          py::arg("xmin") = -1.f, py::arg("ymin") = -1.f, py::arg("zmin") = -1.f,
          py::arg("xmax") = 1.f, py::arg("ymax") = 1.f, py::arg("zmax") = 1.f,
          py::arg("frequency") = 1.f, py::arg("lineWidth") = 3.f, py::arg("marks") = true,
          "Constructor.")
      .def(
          "setAxisLimits", &CAxis::setAxisLimits,
          "Sets the axis limits (xmin, ymin, zmin, xmax, ymax, zmax).")
      .def("setFrequency", &CAxis::setFrequency, "Changes the frequency of the \"ticks\".")
      .def("getFrequency", &CAxis::getFrequency, "Returns the spacing between tick marks.")
      .def("setTextScale", &CAxis::setTextScale, "Sets the size of text labels (default: 0.25).")
      .def("getTextScale", &CAxis::getTextScale, "Returns the size of text labels.")
      .def(
          "enableTickMarks", py::overload_cast<bool>(&CAxis::enableTickMarks),
          "Shows or hides the tick marks.");

  // 11. CBox
  py::class_<CBox, CVisualObject, std::shared_ptr<CBox>>(
      m, "CBox", "A solid or wireframe box, given its two opposite corners.",
      py::multiple_inheritance())
      .def(py::init<>(), "Default constructor.")
      .def(
          py::init<const mrpt::math::TPoint3D&, const mrpt::math::TPoint3D&, bool, float>(),
          py::arg("corner1"), py::arg("corner2"), py::arg("is_wireframe") = false,
          py::arg("lineWidth") = 1.0f,
          "Builds the box from two opposite corners, solid or wireframe, and line width.")
      .def(
          "setBoxCorners", &CBox::setBoxCorners,
          "Set the position and size of the box, from two corners in 3D.")
      .def(
          "getBoxCorners",
          [](const CBox& self)
          {
            mrpt::math::TPoint3D c1;
            mrpt::math::TPoint3D c2;
            self.getBoxCorners(c1, c2);
            return py::make_tuple(c1, c2);
          },
          "Get the current box corners.")
      .def(
          "setWireframe", &CBox::setWireframe,
          "Sets wireframe rendering mode (true) or solid mode (false, default)")
      .def("isWireframe", &CBox::isWireframe, "Returns true if wireframe mode is enabled.")
      .def(
          "enableBoxBorder", &CBox::enableBoxBorder,
          "Enable/disable drawing a border around solid boxes.")
      .def(
          "setBoxBorderColor", &CBox::setBoxBorderColor, "color"_a,
          "Color of the box edges, drawn if enableBoxBorder()");

  // 12. CSphere
  py::class_<CSphere, CVisualObject, std::shared_ptr<CSphere>>(
      m, "CSphere", "A solid or wire-frame sphere.", py::multiple_inheritance())
      .def(py::init<float, int>(), py::arg("radius") = 1.0f, py::arg("nDivs") = 20, "Constructor.")
      .def("setRadius", &CSphere::setRadius, "Sets the sphere radius.")
      .def("getRadius", &CSphere::getRadius, "Returns the sphere radius.")
      .def(
          "setNumberDivs", &CSphere::setNumberDivs,
          "Sets the number of slices and stacks used to render the sphere.");

  // 13. CCylinder
  py::class_<CCylinder, CVisualObject, std::shared_ptr<CCylinder>>(
      m, "CCylinder", "A cylinder or cone whose base lies in the XY plane.",
      py::multiple_inheritance())
      .def(py::init<>(), "Default constructor: unit cylinder.")
      .def(
          py::init<float, float, float, int>(), py::arg("baseRadius"), py::arg("topRadius"),
          py::arg("height") = 1.0f, py::arg("slices") = 20, "Constructor with parameters.")
      .def(
          "setRadius", &CCylinder::setRadius,
          "Sets both radii to a single value, configuring the object as a cylinder.")
      .def("setRadii", &CCylinder::setRadii, "Sets both radii independently.")
      .def("setHeight", &CCylinder::setHeight, "Changes cylinder's height.")
      .def("getHeight", &CCylinder::getHeight, "Gets the cylinder's height.");

  // 14. CArrow
  py::class_<CArrow, CVisualObject, std::shared_ptr<CArrow>>(
      m, "CArrow", "A 3D arrow.", py::multiple_inheritance())
      .def(py::init<>(), "Default constructor.")
      .def(
          "setArrowEnds",
          [](CArrow& self, float x0, float y0, float z0, float x1, float y1, float z1)
          { self.setArrowEnds(x0, y0, z0, x1, y1, z1); },
          py::arg("x0"), py::arg("y0"), py::arg("z0"), py::arg("x1"), py::arg("y1"), py::arg("z1"),
          "Sets the arrow start (x0, y0, z0) and end (x1, y1, z1) points.")
      .def(
          "setHeadRatio", &CArrow::setHeadRatio,
          "Sets the length of the arrow head, relative to the arrow length.")
      .def("setSmallRadius", &CArrow::setSmallRadius, "Sets the radius of the arrow body.");

  // 15. CText
  py::class_<CText, CVisualObject, std::shared_ptr<CText>>(
      m, "CText", "A 2D text label at a 3D position, always facing the camera.",
      py::multiple_inheritance())
      .def(py::init<>(), "Default constructor.")
      .def(py::init<const std::string&>(), py::arg("text"), "Builds the label with the given text.")
      .def("setString", &CText::setString, "Sets the text to display.")
      .def("getString", &CText::getString, "Return the current text associated to this label.")
      .def("setFont", &CText::setFont, "Sets the font among \"sans\", \"serif\", \"mono\".");

  // 16. CText3D
  py::class_<CText3D, CVisualObject, std::shared_ptr<CText3D>>(
      m, "CText3D",
      "A 3D text (rendered with OpenGL primitives), with selectable font face and drawing style.",
      py::multiple_inheritance())
      .def(
          py::init<const std::string&, const std::string&, float>(), py::arg("text") = "",
          py::arg("fontName") = "sans", py::arg("scale") = 1.0f,
          "Builds a 3D text from its string, font name and scale.")
      .def("setString", &CText3D::setString, "Sets the displayed string.")
      .def(
          "getString", &CText3D::getString,
          "Returns the currently text associated to this object.");

  // 17. CSetOfLines
  py::class_<CSetOfLines, CVisualObject, std::shared_ptr<CSetOfLines>>(
      m, "CSetOfLines",
      "A set of independent lines (or segments), one line with its own start and end positions "
      "(X,Y,Z).",
      py::multiple_inheritance())
      .def(py::init<>(), "Constructor.")
      .def("clear", &CSetOfLines::clear, "Clear the list of segments.")
      .def(
          "appendLine",
          static_cast<void (CSetOfLines::*)(double, double, double, double, double, double)>(
              &CSetOfLines::appendLine),
          py::arg("x0"), py::arg("y0"), py::arg("z0"), py::arg("x1"), py::arg("y1"), py::arg("z1"),
          "Appends a segment given its end points (x0, y0, z0, x1, y1, z1).")
      .def(
          "appendLine",
          static_cast<void (CSetOfLines::*)(const mrpt::math::TSegment3D&)>(
              &CSetOfLines::appendLine),
          "Appends a segment.")
      .def("__len__", [](const CSetOfLines& self) { return self.size(); });

  // 18. CSimpleLine
  py::class_<CSimpleLine, CVisualObject, std::shared_ptr<CSimpleLine>>(
      m, "CSimpleLine", "A line segment.", py::multiple_inheritance())
      .def(py::init<>(), "Default constructor.")
      .def(
          "setLineCoords",
          py::overload_cast<float, float, float, float, float, float>(&CSimpleLine::setLineCoords),
          py::arg("x0"), py::arg("y0"), py::arg("z0"), py::arg("x1"), py::arg("y1"), py::arg("z1"),
          "Sets the line end points (x0, y0, z0, x1, y1, z1).")
      .def("getLineStart", &CSimpleLine::getLineStart, "Returns the line start point.")
      .def("getLineEnd", &CSimpleLine::getLineEnd, "Returns the line end point.");

  // 19. CEllipsoid3D
  py::class_<CEllipsoid3D, CVisualObject, std::shared_ptr<CEllipsoid3D>>(
      m, "CEllipsoid3D", "A 3D ellipsoid, centered at zero with respect to this object pose.",
      py::multiple_inheritance())
      .def(py::init<>(), "Default constructor.")
      .def(
          "setCovMatrix",
          [](CEllipsoid3D& self, const mrpt::math::CMatrixDouble33& cov)
          { self.setCovMatrix(cov); },
          "Like setCovMatrixAndMean(), for mean=zero.")
      .def(
          "setQuantiles", &CEllipsoid3D::setQuantiles,
          "Changes the scale of the \"sigmas\" for drawing the ellipse/ellipsoid (default=3, ~97 "
          "or ~98% CI); the exact mathematical meaning is: This value of \"quantiles\" q should be "
          "set to the square root of the chi-squared inverse cdf corresponding to the desired "
          "confidence interval.")
      .def(
          "set3DsegmentsCount", &CEllipsoid3D::set3DsegmentsCount,
          "The number of segments of a 3D ellipse (in both \"axes\") (default=20)");

  // 20. CEllipsoid2D
  py::class_<CEllipsoid2D, CVisualObject, std::shared_ptr<CEllipsoid2D>>(
      m, "CEllipsoid2D",
      "A 2D ellipse on the XY plane, centered at the origin of this object pose.",
      py::multiple_inheritance())
      .def(py::init<>(), "Default constructor.")
      .def(
          "setCovMatrix",
          [](CEllipsoid2D& self, const mrpt::math::CMatrixDouble22& cov)
          { self.setCovMatrix(cov); },
          "Like setCovMatrixAndMean(), for mean=zero.")
      .def(
          "setQuantiles", &CEllipsoid2D::setQuantiles,
          "Changes the scale of the \"sigmas\" for drawing the ellipse/ellipsoid (default=3, ~97 "
          "or ~98% CI); the exact mathematical meaning is: This value of \"quantiles\" q should be "
          "set to the square root of the chi-squared inverse cdf corresponding to the desired "
          "confidence interval.");

  // 21. CPointCloudColoured (complement to existing CPointCloud)
  py::class_<CPointCloudColoured, CVisualObject, std::shared_ptr<CPointCloudColoured>>(
      m, "CPointCloudColoured", "A cloud of points, each one with an individual color (R,G,B,A).",
      py::multiple_inheritance())
      .def(py::init<>(), "Default constructor.")
      .def("clear", &CPointCloudColoured::clear, "Erase all the points.")
      .def(
          "push_back",
          [](CPointCloudColoured& self, float x, float y, float z, float r, float g, float b,
             float a) { self.push_back(x, y, z, r, g, b, a); },
          py::arg("x"), py::arg("y"), py::arg("z"), py::arg("r") = 1.0f, py::arg("g") = 1.0f,
          py::arg("b") = 1.0f, py::arg("a") = 1.0f, "Inserts a new point into the point cloud.")
      .def("size", &CPointCloudColoured::size, "Return the number of points.")
      .def("__len__", [](const CPointCloudColoured& self) { return self.size(); })
      .def(
          "setPointSize", &CPointCloudColoured::setPointSize, "pointSize"_a,
          "Point size, in pixels")
      .def(
          "getPointSize", &CPointCloudColoured::getPointSize,
          "Returns the rendered point size, in pixels.");

  // 22. stock_objects submodule
  // TTriangle
  py::class_<TTriangle::Vertex>(
      m, "TTriangleVertex",
      "One vertex of a TTriangle: position, color, normal and texture coordinates.")
      .def(py::init<>(), "Default constructor.")
      .def(
          "setColor", &TTriangle::Vertex::setColor, py::arg("color"),
          "Set vertex color from TColor");

  py::class_<TTriangle>(
      m, "TTriangle",
      "A triangle (float coordinates) with RGBA colors (u8) and UV (texture coordinates) for each "
      "vertex.")
      .def(py::init<>(), "Default constructor.")
      .def(
          py::init<
              const mrpt::math::TPoint3Df&, const mrpt::math::TPoint3Df&,
              const mrpt::math::TPoint3Df&>(),
          py::arg("p1"), py::arg("p2"), py::arg("p3"),
          "Constructor from 3 points (default normals are computed)")
      .def(
          "computeNormals", &TTriangle::computeNormals,
          "Compute the three normals from the cross-product of \"v01 x v02\".")
      .def_readwrite("vertices", &TTriangle::vertices);

  // CDisk
  py::class_<CDisk, CVisualObject, std::shared_ptr<CDisk>>(
      m, "CDisk", "A planar disk in the XY plane.", py::multiple_inheritance())
      .def(py::init<>(), "Constructor.")
      .def(
          py::init<float, float, uint32_t>(), py::arg("out_radius"), py::arg("in_radius"),
          py::arg("slices") = 50U,
          "Builds the disk from its outer and inner radii and number of slices.")
      .def(
          "setDiskRadius", &CDisk::setDiskRadius, py::arg("out_radius"),
          py::arg("in_radius") = 0.0f, "Sets the outer and inner radii.")
      .def_property_readonly("in_radius", &CDisk::getInRadius, "Inner radius.")
      .def_property_readonly("out_radius", &CDisk::getOutRadius, "Outer radius.")
      .def(
          "setSlicesCount", &CDisk::setSlicesCount, py::arg("N"),
          "Sets the number of slices (at least 3; default: 50).");

  // CFrustum
  py::class_<CFrustum, CVisualObject, std::shared_ptr<CFrustum>>(
      m, "CFrustum",
      "A solid or wireframe frustum in 3D (a rectangular truncated pyramid), with arbitrary "
      "(possibly assymetric) field-of-view angles.",
      py::multiple_inheritance())
      .def(py::init<>(), "Basic empty constructor. Set all parameters to default.")
      .def(
          py::init<float, float, float, float, float, bool, bool>(), py::arg("near_distance"),
          py::arg("far_distance"), py::arg("horz_FOV_degrees"), py::arg("vert_FOV_degrees"),
          py::arg("lineWidth") = 1.0f, py::arg("draw_lines") = true, py::arg("draw_planes") = false,
          "Constructor with some parameters.")
      .def(
          "setNearFarPlanes", &CFrustum::setNearFarPlanes, py::arg("near"), py::arg("far"),
          "Changes distance of near & far planes.")
      .def(
          "setHorzFOV", &CFrustum::setHorzFOV, py::arg("fov_degrees"),
          "Changes horizontal FOV (symmetric)")
      .def(
          "setVertFOV", &CFrustum::setVertFOV, py::arg("fov_degrees"),
          "Changes vertical FOV (symmetric)")
      .def_property_readonly(
          "near_plane", &CFrustum::getNearPlaneDistance, "Distance to the near plane.")
      .def_property_readonly(
          "far_plane", &CFrustum::getFarPlaneDistance, "Distance to the far plane.")
      .def_property_readonly(
          "horz_fov", &CFrustum::getHorzFOV, "Horizontal field of view, in degrees.")
      .def_property_readonly(
          "vert_fov", &CFrustum::getVertFOV, "Vertical field of view, in degrees.")
      .def(
          "setPlaneColor", &CFrustum::setPlaneColor, py::arg("color"),
          "Sets the color of the planes (line color is set with setColor()).");

  // CSetOfTriangles
  py::class_<CSetOfTriangles, CVisualObject, std::shared_ptr<CSetOfTriangles>>(
      m, "CSetOfTriangles",
      "A set of colored triangles, able to draw any solid, arbitrarily complex object without "
      "textures.",
      py::multiple_inheritance())
      .def(py::init<>(), "Default constructor.")
      .def(
          "clearTriangles", &CSetOfTriangles::clearTriangles,
          "Clear this object, removing all triangles.")
      .def("getTrianglesCount", &CSetOfTriangles::getTrianglesCount, "Get triangle count.")
      .def(
          "getTriangle",
          [](const CSetOfTriangles& self, size_t idx)
          {
            TTriangle t;
            self.getTriangle(idx, t);
            return t;
          },
          py::arg("idx"), "Gets the i-th triangle.")
      .def(
          "insertTriangle", &CSetOfTriangles::insertTriangle, py::arg("triangle"),
          "Inserts a triangle into the set.");

  // CVectorField2D
  py::class_<CVectorField2D, CVisualObject, std::shared_ptr<CVectorField2D>>(
      m, "CVectorField2D",
      "A 2D vector field representation, consisting of points and arrows drawn on a plane "
      "(invisible grid).",
      py::multiple_inheritance())
      .def(py::init<>(), "Constructor.")
      .def("clear", &CVectorField2D::clear, "Clear the matrices.")
      .def(
          "setGridLimits", &CVectorField2D::setGridLimits, py::arg("xmin"), py::arg("xmax"),
          py::arg("ymin"), py::arg("ymax"),
          "Set the coordinates of the grid on where the vector field will be drawn using x-y max "
          "and min values.")
      .def(
          "setGridCenterAndCellSize", &CVectorField2D::setGridCenterAndCellSize, py::arg("cx"),
          py::arg("cy"), py::arg("cell_x"), py::arg("cell_y"),
          "Set the coordinates of the grid on where the vector field will be drawn by setting its "
          "center and the cell size.")
      .def(
          "setPointColor", &CVectorField2D::setPointColor, py::arg("R"), py::arg("G"), py::arg("B"),
          py::arg("A") = 1.0f, "Set the point color in the range [0,1].")
      .def(
          "setVectorFieldColor", &CVectorField2D::setVectorFieldColor, py::arg("R"), py::arg("G"),
          py::arg("B"), py::arg("A") = 1.0f, "Set the arrow color in the range [0,1].")
      .def(
          "setVectorField",
          [](CVectorField2D& self, const FloatMatrix& vx, const FloatMatrix& vy)
          {
            auto mx = toCMatrixFloat(vx);
            auto my = toCMatrixFloat(vy);
            self.setVectorField(mx, my);
          },
          py::arg("vx"), py::arg("vy"),
          "Sets the vector components at each grid cell, as 2D float arrays of equal shape.");

  // CVectorField3D
  py::class_<CVectorField3D, CVisualObject, std::shared_ptr<CVectorField3D>>(
      m, "CVectorField3D",
      "A 3D vector field representation, consisting of points and arrows drawn at any spatial "
      "position.",
      py::multiple_inheritance())
      .def(py::init<>(), "Default constructor.")
      .def("clear", &CVectorField3D::clear, "Clear the matrices.")
      .def(
          "setPointColor", &CVectorField3D::setPointColor, py::arg("R"), py::arg("G"), py::arg("B"),
          py::arg("A") = 1.0f, "Set the point color in the range [0,1].")
      .def(
          "setVectorFieldColor", &CVectorField3D::setVectorFieldColor, py::arg("R"), py::arg("G"),
          py::arg("B"), py::arg("A") = 1.0f, "Set the arrow color in the range [0,1].")
      .def(
          "setMaxSpeedForColor", &CVectorField3D::setMaxSpeedForColor, py::arg("s"),
          "Set the max speed associated for the color map ( m_still_color, m_maxspeed_color)")
      .def(
          "setVectorField",
          [](CVectorField3D& self, const FloatMatrix& vx, const FloatMatrix& vy,
             const FloatMatrix& vz)
          {
            auto mx = toCMatrixFloat(vx);
            auto my = toCMatrixFloat(vy);
            auto mz = toCMatrixFloat(vz);
            self.setVectorField(mx, my, mz);
          },
          py::arg("vx"), py::arg("vy"), py::arg("vz"),
          "Sets the vector components at each point, as 2D float arrays of equal shape.")
      .def(
          "setPointCoordinates",
          [](CVectorField3D& self, const FloatMatrix& px, const FloatMatrix& py2,
             const FloatMatrix& pz)
          {
            auto mx = toCMatrixFloat(px);
            auto my = toCMatrixFloat(py2);
            auto mz = toCMatrixFloat(pz);
            self.setPointCoordinates(mx, my, mz);
          },
          py::arg("px"), py::arg("py"), py::arg("pz"),
          "Sets the point coordinates, as 2D float arrays of equal shape.");

  // CMesh
  py::class_<CMesh, CVisualObject, std::shared_ptr<CMesh>>(
      m, "CMesh",
      "A planar (XY) grid where each cell has an associated height and, optionally, a texture map.",
      py::multiple_inheritance())
      .def(
          py::init<bool, float, float, float, float>(), py::arg("enable_transparency") = false,
          py::arg("xMin") = -1.0f, py::arg("xMax") = 1.0f, py::arg("yMin") = -1.0f,
          py::arg("yMax") = 1.0f, "Builds the mesh from its transparency flag and x and y limits.")
      .def(
          "setGridLimits",
          [](CMesh& self, float x0, float x1, float y0, float y1)
          { self.setGridLimits(x0, x1, y0, y1); },
          py::arg("xMin"), py::arg("xMax"), py::arg("yMin"), py::arg("yMax"),
          "Sets the x and y limits of the grid.")
      .def(
          "enableTransparency", &CMesh::enableTransparency, py::arg("v"),
          "Enables or disables transparency.")
      .def(
          "enableWireFrame", &CMesh::enableWireFrame, py::arg("v"),
          "Shows the mesh as wireframe (true) or solid (false).")
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
  py::class_<CColorBar, CVisualObject, std::shared_ptr<CColorBar>>(
      m, "CColorBar",
      "A colorbar indicator. This class renders a colorbar as a 3D object, in the XY plane.",
      py::multiple_inheritance())
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
          py::arg("label_font_size") = 0.05f,
          "Builds a color bar from its colormap, size, color range, value range and label format.")
      .def(
          "setColormap",
          [](CColorBar& self, int colormap)
          { self.setColormap(static_cast<mrpt::img::TColormap>(colormap)); },
          py::arg("colormap"), "Sets the colormap, as the integer value of a mrpt.img.TColormap.")
      .def(
          "setColorAndValueLimits", &CColorBar::setColorAndValueLimits, py::arg("col_min"),
          py::arg("col_max"), py::arg("value_min"), py::arg("value_max"),
          "Sets the color range and the value range it represents.");

  // CMesh3D
  py::class_<CMesh3D, CVisualObject, std::shared_ptr<CMesh3D>>(
      m, "CMesh3D",
      "A 3D mesh composed of triangles and/or quads. A typical usage example would be a 3D model "
      "of an object.",
      py::multiple_inheritance())
      .def(py::init<>(), "Default constructor.")
      .def("enableShowEdges", &CMesh3D::enableShowEdges, py::arg("v"), "Shows or hides the edges.")
      .def("enableShowFaces", &CMesh3D::enableShowFaces, py::arg("v"), "Shows or hides the faces.")
      .def(
          "enableShowVertices", &CMesh3D::enableShowVertices, py::arg("v"),
          "Shows or hides the vertices.")
      .def(
          "enableFaceNormals", &CMesh3D::enableFaceNormals, py::arg("v"),
          "Enables or disables computing normals per face.")
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
            for (py::ssize_t i = 0; i < fv.shape(0); i++)
            {
              fv_v[i] = fv(i);
            }
            self.loadMesh(nv, nf, vpf_v.data(), fv_v.data(), vc.data());
          },
          py::arg("vertices"), py::arg("face_vertices"), py::arg("verts_per_face"),
          "Load a mesh from numpy arrays: vertices (Nx3 float32), face_vertices (M,) int32, "
          "verts_per_face (F,) int32");

  // CMeshFast
  py::class_<CMeshFast, CVisualObject, std::shared_ptr<CMeshFast>>(
      m, "CMeshFast",
      "A planar (XY) grid where each cell has an associated height and, optionally, a texture map.",
      py::multiple_inheritance())
      .def(
          py::init<bool, float, float, float, float>(), py::arg("enable_transparency") = false,
          py::arg("xMin") = -1.0f, py::arg("xMax") = 1.0f, py::arg("yMin") = -1.0f,
          py::arg("yMax") = 1.0f, "Constructor.")
      .def(
          "setGridLimits", &CMeshFast::setGridLimits, py::arg("xmin"), py::arg("xmax"),
          py::arg("ymin"), py::arg("ymax"), "Sets the x and y limits of the grid.")
      .def(
          "enableTransparency", &CMeshFast::enableTransparency, py::arg("v"),
          "Enables or disables transparency.")
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
  py::class_<CTexturedPlane, CVisualObject, std::shared_ptr<CTexturedPlane>>(
      m, "CTexturedPlane", "A 2D plane in the XY plane with a texture image.",
      py::multiple_inheritance())
      .def(
          py::init<float, float, float, float>(), py::arg("x_min") = -1.0f, py::arg("x_max") = 1.0f,
          py::arg("y_min") = -1.0f, py::arg("y_max") = 1.0f,
          "Builds the plane from its x and y limits.")
      .def(
          "setPlaneCorners", &CTexturedPlane::setPlaneCorners, py::arg("xMin"), py::arg("xMax"),
          py::arg("yMin"), py::arg("yMax"),
          "Set the coordinates of the four corners that define the plane on the XY plane.")
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
          },
          "Get the coordinates of the four corners that define the plane on the XY plane.")
      .def(
          "setTextureRepeat", &CTexturedPlane::setTextureRepeat, py::arg("repeatX"),
          py::arg("repeatY"), "Set the number of times the texture repeats in each direction.")
      .def(
          "enableLighting", &CTexturedPlane::enableLighting, py::arg("enable") = true,
          "Enables or disables lighting on the plane.");

  // CSetOfTexturedTriangles
  py::class_<CSetOfTexturedTriangles, CVisualObject, std::shared_ptr<CSetOfTexturedTriangles>>(
      m, "CSetOfTexturedTriangles",
      "A set of textured triangles. This class can be used to draw any solid, arbitrarily complex "
      "object with textures.",
      py::multiple_inheritance())
      .def(
          py::init([]() { return std::make_shared<CSetOfTexturedTriangles>(); }),
          "Default constructor.")
      .def(
          "clearTriangles", [](CSetOfTexturedTriangles& self) { self.clearTriangles(); },
          "Removes all triangles.")
      .def(
          "getTrianglesCount",
          [](const CSetOfTexturedTriangles& self) { return self.getTrianglesCount(); },
          "Returns the number of triangles.")
      .def(
          "getTriangle",
          [](const CSetOfTexturedTriangles& self, size_t idx) -> mrpt::viz::TTriangle
          { return self.getTriangle(idx); },
          py::arg("idx"), "Returns the i-th triangle.")
      .def(
          "insertTriangle",
          [](CSetOfTexturedTriangles& self, const mrpt::viz::TTriangle& t)
          { self.insertTriangle(t); },
          py::arg("triangle"), "Appends a triangle.");

  // CPolyhedron
  py::class_<CPolyhedron, CVisualObject, std::shared_ptr<CPolyhedron>>(
      m, "CPolyhedron",
      "This class represents arbitrary polyhedra. The class includes a set of static methods to "
      "create common polyhedrons.",
      py::multiple_inheritance())
      .def_static(
          "CreateTetrahedron", &CPolyhedron::CreateTetrahedron, py::arg("radius"),
          "Creates a regular tetrahedron (see http://en.wikipedia.org/wiki/Tetrahedron).")
      .def_static(
          "CreateHexahedron", &CPolyhedron::CreateHexahedron, py::arg("radius"),
          "Creates a regular cube, also called hexahedron (see "
          "http://en.wikipedia.org/wiki/Hexahedron).")
      .def_static(
          "CreateOctahedron", &CPolyhedron::CreateOctahedron, py::arg("radius"),
          "Creates a regular octahedron (see http://en.wikipedia.org/wiki/Octahedron).")
      .def_static(
          "CreateDodecahedron", &CPolyhedron::CreateDodecahedron, py::arg("radius"),
          "Creates a regular dodecahedron (see http://en.wikipedia.org/wiki/Dodecahedron).")
      .def_static(
          "CreateIcosahedron", &CPolyhedron::CreateIcosahedron, py::arg("radius"),
          "Creates a regular icosahedron (see http://en.wikipedia.org/wiki/Icosahedron).")
      .def_static(
          "CreateTruncatedTetrahedron", &CPolyhedron::CreateTruncatedTetrahedron, py::arg("radius"),
          "Creates a truncated tetrahedron, consisting of four triangular faces and for hexagonal "
          "ones (see http://en.wikipedia.org/wiki/Truncated_tetrahedron).")
      .def_static(
          "CreateTruncatedHexahedron", &CPolyhedron::CreateTruncatedHexahedron, py::arg("radius"),
          "Creates a truncated hexahedron, with six octogonal faces and eight triangular ones (see "
          "http://en.wikipedia.org/wiki/Truncated_hexahedron).")
      .def_static(
          "CreateTruncatedOctahedron", &CPolyhedron::CreateTruncatedOctahedron, py::arg("radius"),
          "Creates a truncated octahedron, with eight hexagons and eight squares (see "
          "http://en.wikipedia.org/wiki/Truncated_octahedron).")
      .def_static(
          "CreateTruncatedIcosahedron", &CPolyhedron::CreateTruncatedIcosahedron, py::arg("radius"),
          "Creates a truncated icosahedron, consisting of 20 hexagons and 12 pentagons.")
      .def_static(
          "CreateCuboctahedron", &CPolyhedron::CreateCuboctahedron, py::arg("radius"),
          "Creates a cuboctahedron, consisting of six square faces and eight triangular ones (see "
          "http://en.wikipedia.org/wiki/Cuboctahedron).")
      .def_static(
          "CreateIcosidodecahedron",
          py::overload_cast<double, bool>(&CPolyhedron::CreateIcosidodecahedron), py::arg("radius"),
          py::arg("type") = true,
          "Creates an icosidodecahedron, with 12 pentagons and 20 triangles (see "
          "http://en.wikipedia.org/wiki/Icosidodecahedron).");

  // COrbitCameraController
  py::class_<COrbitCameraController>(
      m, "COrbitCameraController", "Framework-agnostic orbit/pan/zoom camera controller.")
      .def(py::init<>(), "Default constructor.")
      .def(
          "setCameraPointing",
          py::overload_cast<float, float, float>(&COrbitCameraController::setCameraPointing),
          py::arg("x"), py::arg("y"), py::arg("z"), "Sets the point the camera looks at (x, y, z).")
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
      .def(
          "setZoomDistance", &COrbitCameraController::setZoomDistance, py::arg("d"),
          "Sets the camera distance to the point it looks at.")
      .def(
          "setAzimuthDegrees", &COrbitCameraController::setAzimuthDegrees, py::arg("deg"),
          "Sets the camera azimuth angle, in degrees.")
      .def(
          "setElevationDegrees", &COrbitCameraController::setElevationDegrees, py::arg("deg"),
          "Sets the camera elevation angle, in degrees.")
      .def(
          "applyTo", &COrbitCameraController::applyTo, py::arg("cam"),
          "Writes the current orbit parameters into cam.")
      .def(
          "setFrom", &COrbitCameraController::setFrom, py::arg("cam"),
          "Initialises the controller from an existing CCamera.")
      .def(
          "onMouseMove", &COrbitCameraController::onMouseMove, py::arg("x"), py::arg("y"),
          py::arg("buttons"), py::arg("modifiers"),
          "Call on mouse-move events while any button is held.")
      .def(
          "onMouseButton", &COrbitCameraController::onMouseButton, py::arg("x"), py::arg("y"),
          py::arg("button"), py::arg("down"), "Call on button press/release.")
      .def(
          "onScroll", &COrbitCameraController::onScroll, py::arg("delta"), py::arg("modifiers"),
          "Call on scroll-wheel events.");

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

  py::class_<COctoMapVoxels, CVisualObject, std::shared_ptr<COctoMapVoxels>>(
      m, "COctoMapVoxels", "Renders voxels, typically from a 3D octomap.",
      py::multiple_inheritance())
      .def(py::init<>(), "Constructor.")
      .def("clear", &COctoMapVoxels::clear, "Clears everything.")
      .def(
          "setVisualizationMode", &COctoMapVoxels::setVisualizationMode, py::arg("mode"),
          "Select the visualization mode. To have any effect, this method has to be called before "
          "loading the octomap.")
      .def(
          "enableLights", &COctoMapVoxels::enableLights, py::arg("enable"),
          "Can be used to enable/disable the effects of lighting in this object.")
      .def(
          "enableCubeTransparency", &COctoMapVoxels::enableCubeTransparency, py::arg("enable"),
          "Enables or disables using the alpha channel of the voxel colors.")
      .def(
          "showGridLines", &COctoMapVoxels::showGridLines, py::arg("show"),
          "Shows/hides the grid lines.")
      .def(
          "showVoxels", &COctoMapVoxels::showVoxels, py::arg("voxel_set"), py::arg("show"),
          "Shows or hides the voxels of one voxel set.")
      .def(
          "showVoxelsAsPoints", &COctoMapVoxels::showVoxelsAsPoints, py::arg("enable"),
          "For quick renders: render voxels as points instead of cubes.")
      .def(
          "getVoxelCount", &COctoMapVoxels::getVoxelCount, py::arg("set_index"),
          "Returns the total count of voxels in one voxel set.")
      .def(
          "getVoxelSetCount", &COctoMapVoxels::getVoxelSetCount,
          "Returns the number of voxel sets.")
      .def(
          "resizeVoxelSets", &COctoMapVoxels::resizeVoxelSets, py::arg("n"),
          "Sets the number of voxel sets.")
      .def(
          "resizeVoxels", &COctoMapVoxels::resizeVoxels, py::arg("set_index"), py::arg("n"),
          "Sets the number of voxels in one voxel set.")
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
          py::arg("r") = 200, py::arg("g") = 200, py::arg("b") = 200, py::arg("a") = 255,
          "Appends a voxel (center, side length and RGBA color) to a voxel set.");

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
  py::class_<CSkyBox, CVisualObject, std::shared_ptr<CSkyBox>>(
      m, "CSkyBox",
      "A Sky Box: 6 textures that are always rendered at \"infinity\" to give the impression of "
      "the scene to be much larger.",
      py::multiple_inheritance())
      .def(py::init<>(), "Default constructor.")
      .def(
          "assignImage",
          py::overload_cast<const CUBE_TEXTURE_FACE, const mrpt::img::CImage&>(
              &CSkyBox::assignImage),
          py::arg("face"), py::arg("img"),
          "Assigns a texture. It is mandatory to assign all 6 faces before initializing/rendering "
          "the texture.");

  // TLightType enum
  py::enum_<TLightType>(m, "TLightType")
      .value("Directional", TLightType::Directional)
      .value("Point", TLightType::Point)
      .value("Spot", TLightType::Spot)
      .export_values();

  // TLight struct
  py::class_<TLight>(m, "TLight", "A single light source (directional, point, or spot).")
      .def(py::init<>(), "Default constructor.")
      .def_readwrite("type", &TLight::type)
      .def_readwrite("diffuse", &TLight::diffuse)
      .def_readwrite("specular", &TLight::specular)
      .def_readwrite("direction", &TLight::direction)
      .def_readwrite("position", &TLight::position)
      .def_readwrite("attenuation_constant", &TLight::attenuation_constant)
      .def_readwrite("attenuation_linear", &TLight::attenuation_linear)
      .def_readwrite("attenuation_quadratic", &TLight::attenuation_quadratic)
      .def_readwrite(
          "range", &TLight::range,
          "Maximum reach of Point/Spot lights [m], where the light fades to zero (0=unlimited).")
      .def_readwrite(
          "cast_shadows", &TLight::cast_shadows,
          "Whether this Point/Spot light casts shadows (cube shadow map, default: False).")
      .def_readwrite("spot_inner_cutoff_deg", &TLight::spot_inner_cutoff_deg)
      .def_readwrite("spot_outer_cutoff_deg", &TLight::spot_outer_cutoff_deg)
      .def_static(
          "Directional",
          [](const mrpt::math::TVector3Df& dir, float r, float g, float b, float diffuse,
             float specular)
          { return TLight::Directional(dir, mrpt::img::TColorf(r, g, b), diffuse, specular); },
          py::arg("dir"), py::arg("r") = 1.0f, py::arg("g") = 1.0f, py::arg("b") = 1.0f,
          py::arg("diffuse") = 0.8f, py::arg("specular") = 0.95f,
          "Factory: creates a directional light.");

  py::class_<CLight, CVisualObject, std::shared_ptr<CLight>>(
      m, "CLight",
      "A light source placed in the scene graph: its position and direction are relative to the "
      "pose of this object and its parents. It is switched on and off with setVisibility().",
      py::multiple_inheritance())
      .def(py::init<>(), "Default constructor.")
      .def(
          py::init<const TLight&>(), "light"_a,
          "Constructor from the light parameters, in the local frame of this object.")
      .def_property(
          "light", [](const CLight& self) { return self.light(); },
          [](CLight& self, const TLight& l) { self.light(l); },
          "The light parameters, in the local frame of this object.");

  // Generalized ellipsoids: CEllipsoidInverseDepth2D, CEllipsoidInverseDepth3D,
  // CEllipsoidRangeBearing2D. CGeneralizedEllipsoidTemplate<N> has a protected destructor so
  // we cannot register the base; bind the concrete classes directly under CVisualObject instead.
  py::class_<CEllipsoidInverseDepth2D, CVisualObject, std::shared_ptr<CEllipsoidInverseDepth2D>>(
      m, "CEllipsoidInverseDepth2D",
      "An uncertainty ellipse of an (inverse range, yaw) variable, drawn in 2D Cartesian space.",
      py::multiple_inheritance())
      .def(py::init<>(), "Default constructor.")
      .def(
          "setQuantiles", &CEllipsoidInverseDepth2D::setQuantiles, py::arg("q"),
          "Changes the scale of the \"sigmas\" for drawing the ellipse/ellipsoid (default=3, ~97 "
          "or ~98% CI); the exact mathematical meaning is: This value of \"quantiles\" q should be "
          "set to the square root of the chi-squared inverse cdf corresponding to the desired "
          "confidence interval.")
      .def(
          "getQuantiles", &CEllipsoidInverseDepth2D::getQuantiles,
          "Returns the number of sigmas of the drawn ellipse (see setQuantiles()).")
      .def(
          "setCovMatrix",
          [](CEllipsoidInverseDepth2D& self, const mrpt::math::CMatrixDouble22& cov)
          { self.setCovMatrix(cov); },
          "Like setCovMatrixAndMean(), for mean=zero.")
      .def(
          "setUnderflowMaxRange", &CEllipsoidInverseDepth2D::setUnderflowMaxRange,
          py::arg("maxRange"),
          "Sets the range used for points of the ellipsoid that fall in negative ranges (default: "
          "1e6).")
      .def(
          "getUnderflowMaxRange", &CEllipsoidInverseDepth2D::getUnderflowMaxRange,
          "Returns the range used for points of the ellipsoid that fall in negative ranges.");

  py::class_<CEllipsoidInverseDepth3D, CVisualObject, std::shared_ptr<CEllipsoidInverseDepth3D>>(
      m, "CEllipsoidInverseDepth3D",
      "An uncertainty ellipsoid of an (inverse range, yaw, pitch) variable, drawn in 3D Cartesian "
      "space.",
      py::multiple_inheritance())
      .def(py::init<>(), "Constructor.")
      .def(
          "setQuantiles", &CEllipsoidInverseDepth3D::setQuantiles, py::arg("q"),
          "Changes the scale of the \"sigmas\" for drawing the ellipse/ellipsoid (default=3, ~97 "
          "or ~98% CI); the exact mathematical meaning is: This value of \"quantiles\" q should be "
          "set to the square root of the chi-squared inverse cdf corresponding to the desired "
          "confidence interval.")
      .def(
          "getQuantiles", &CEllipsoidInverseDepth3D::getQuantiles,
          "Returns the number of sigmas of the drawn ellipsoid (see setQuantiles()).")
      .def(
          "setCovMatrix",
          [](CEllipsoidInverseDepth3D& self, const mrpt::math::CMatrixDouble33& cov)
          { self.setCovMatrix(cov); },
          "Like setCovMatrixAndMean(), for mean=zero.")
      .def(
          "setUnderflowMaxRange", &CEllipsoidInverseDepth3D::setUnderflowMaxRange,
          py::arg("maxRange"),
          "Sets the range used for points of the ellipsoid that fall in negative ranges (default: "
          "1e6).")
      .def(
          "getUnderflowMaxRange", &CEllipsoidInverseDepth3D::getUnderflowMaxRange,
          "Returns the range used for points of the ellipsoid that fall in negative ranges.");

  py::class_<CEllipsoidRangeBearing2D, CVisualObject, std::shared_ptr<CEllipsoidRangeBearing2D>>(
      m, "CEllipsoidRangeBearing2D",
      "An uncertainty ellipse of a (range, bearing) variable, drawn in 2D Cartesian space.",
      py::multiple_inheritance())
      .def(py::init<>(), "Constructor.")
      .def(
          "setQuantiles", &CEllipsoidRangeBearing2D::setQuantiles, py::arg("q"),
          "Changes the scale of the \"sigmas\" for drawing the ellipse/ellipsoid (default=3, ~97 "
          "or ~98% CI); the exact mathematical meaning is: This value of \"quantiles\" q should be "
          "set to the square root of the chi-squared inverse cdf corresponding to the desired "
          "confidence interval.")
      .def(
          "getQuantiles", &CEllipsoidRangeBearing2D::getQuantiles,
          "Returns the number of sigmas of the drawn ellipse (see setQuantiles()).")
      .def(
          "setCovMatrix",
          [](CEllipsoidRangeBearing2D& self, const mrpt::math::CMatrixDouble22& cov)
          { self.setCovMatrix(cov); },
          "Like setCovMatrixAndMean(), for mean=zero.");

  // CAnimatedAssimpModel
  py::class_<CAnimatedAssimpModel, CAssimpModel, std::shared_ptr<CAnimatedAssimpModel>>(
      m, "CAnimatedAssimpModel", "Extension of CAssimpModel with skeletal animation support.",
      py::multiple_inheritance())
      .def(py::init<>(), "Default constructor.")
      .def(
          "loadScene",
          [](CAnimatedAssimpModel& self, const std::string& fn, int flags)
          { self.loadScene(fn, flags); },
          py::arg("file_name"),
          py::arg("flags") =
              (CAssimpModel::LoadFlags::RealTimeMaxQuality | CAssimpModel::LoadFlags::FlipUVs |
               CAssimpModel::LoadFlags::Verbose),
          "Loads a 3D scene and extracts skeleton/animation data.")
      .def(
          "setAnimationTime", &CAnimatedAssimpModel::setAnimationTime, py::arg("t_seconds"),
          "Set current animation time in seconds. Updates all bone transforms for the active "
          "animation and rebuilds the mesh geometry with skinned positions.")
      .def(
          "getAnimationDuration", &CAnimatedAssimpModel::getAnimationDuration,
          py::arg("anim_idx") = 0, "Get animation duration in seconds.")
      .def(
          "getAnimationCount", &CAnimatedAssimpModel::getAnimationCount,
          "Get number of animations in the model.")
      .def(
          "getAnimationName", &CAnimatedAssimpModel::getAnimationName, py::arg("anim_idx"),
          "Get animation name by index.")
      .def(
          "setActiveAnimation",
          py::overload_cast<const std::string&>(&CAnimatedAssimpModel::setActiveAnimation),
          py::arg("anim_name"), "Select which animation to play (by index).")
      .def(
          "setActiveAnimationByIndex",
          py::overload_cast<size_t>(&CAnimatedAssimpModel::setActiveAnimation), py::arg("idx"),
          "Select which animation to play (by index).")
      .def(
          "setLooping", &CAnimatedAssimpModel::setLooping, py::arg("loop"),
          "Enable/disable animation looping.");

  auto stock = m.def_submodule("stock_objects", "Pre-built 3D objects");
  stock.def(
      "CornerXYZ", &mrpt::viz::stock_objects::CornerXYZ, py::arg("scale") = 1.0f,
      "Returns three arrows for the X, Y, Z axes of a 3D frame.");
  stock.def(
      "CornerXYZSimple", &mrpt::viz::stock_objects::CornerXYZSimple, py::arg("scale") = 1.0f,
      py::arg("lineWidth") = 1.0f,
      "Returns three lines for the X, Y, Z axes of a 3D frame (faster to render than CornerXYZ).");
  stock.def(
      "CornerXYZEye", &mrpt::viz::stock_objects::CornerXYZEye,
      "Returns three arrows for the X, Y, Z axes, with the Z arrowhead at the origin (to show a "
      "camera pose).");
  stock.def(
      "CornerXYSimple", &mrpt::viz::stock_objects::CornerXYSimple, py::arg("scale") = 1.0f,
      py::arg("lineWidth") = 1.0f, "Returns two lines for the X, Y axes of a 2D frame.");
  stock.def(
      "RobotPioneer", &mrpt::viz::stock_objects::RobotPioneer,
      "Returns a 3D model of a Pioneer II mobile robot.");
  stock.def(
      "RobotRhodon", &mrpt::viz::stock_objects::RobotRhodon,
      "Returns a 3D model of the Rhodon mobile robot.");
  stock.def(
      "RobotGiraff", &mrpt::viz::stock_objects::RobotGiraff,
      "Returns a 3D model of the Giraff mobile robot.");
  stock.def(
      "BumblebeeCamera", &mrpt::viz::stock_objects::BumblebeeCamera,
      "Returns a 3D model of a Bumblebee stereo camera.");
  stock.def(
      "Hokuyo_URG", &mrpt::viz::stock_objects::Hokuyo_URG,
      "Returns a 3D model of a Hokuyo URG laser scanner.");
  stock.def(
      "Hokuyo_UTM", &mrpt::viz::stock_objects::Hokuyo_UTM,
      "Returns a 3D model of a Hokuyo UTM laser scanner.");

  // -------------------------------------------------------------------------
  // 3D representations of pose PDFs
  // -------------------------------------------------------------------------
  m.def(
      "posePDF2opengl",
      [](const mrpt::poses::CPosePDF& pdf) { return CSetOfObjects::posePDF2opengl(pdf); }, "pdf"_a,
      "Returns a 3D representation of a 2D pose PDF (ellipses, particles, ...)");
  m.def(
      "posePDF2opengl",
      [](const mrpt::poses::CPose3DPDF& pdf) { return CSetOfObjects::posePDF2opengl(pdf); },
      "pdf"_a, "Returns a 3D representation of a 3D pose PDF (ellipsoids, particles, ...)");
}