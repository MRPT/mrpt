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

#include <mrpt/img/CImage.h>
#include <mrpt/math/CMatrixF.h>
#include <mrpt/opengl/CFBORender.h>
#include <mrpt/viz/CCamera.h>
#include <mrpt/viz/Scene.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace mrpt::opengl;
using namespace pybind11::literals;

namespace
{
using DepthMatrix = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

DepthMatrix toNumpyDepth(const mrpt::math::CMatrixFloat& depth) { return depth.asEigen(); }
}  // namespace

PYBIND11_MODULE(_bindings, m)
{
  // CFBORender::Parameters
  py::class_<CFBORender::Parameters>(
      m, "CFBORenderParameters", "Parameters for CFBORender constructor.")
      .def(
          py::init<unsigned int, unsigned int>(), py::arg("width") = 800, py::arg("height") = 600,
          "Builds the parameters for the given image size.")
      .def_readwrite("width", &CFBORender::Parameters::width)
      .def_readwrite("height", &CFBORender::Parameters::height)
      .def_readwrite("raw_depth", &CFBORender::Parameters::raw_depth)
      .def_readwrite("create_EGL_context", &CFBORender::Parameters::create_EGL_context)
      .def_readwrite("deviceIndexToUse", &CFBORender::Parameters::deviceIndexToUse)
      .def_readwrite("contextMajorVersion", &CFBORender::Parameters::contextMajorVersion)
      .def_readwrite("contextMinorVersion", &CFBORender::Parameters::contextMinorVersion)
      .def_readwrite("contextDebug", &CFBORender::Parameters::contextDebug);

  // CFBORender
  py::class_<CFBORender>(
      m, "CFBORender", "Render 3D scenes off-screen directly to RGB and/or RGB+D images.")
      .def(
          py::init<unsigned int, unsigned int>(), py::arg("width") = 800, py::arg("height") = 600,
          "Convenience constructor with just dimensions.")
      .def(
          "setCamera", &CFBORender::setCamera, py::arg("camera"),
          "Set the camera to use for rendering, overriding the scene's viewport camera.")
      .def(
          "clearCameraOverride", &CFBORender::clearCameraOverride,
          "Clear any camera override, reverting to using the scene's viewport camera.")
      .def(
          "hasCameraOverride", &CFBORender::hasCameraOverride,
          "Returns true if a camera override is set.")
      .def("width", &CFBORender::width, "Returns the current render width in pixels.")
      .def("height", &CFBORender::height, "Returns the current render height in pixels.")
      .def(
          "invalidateCompiledScene", &CFBORender::invalidateCompiledScene,
          "Force recompilation of the scene on next render.")
      .def(
          "render_RGB",
          [](CFBORender& self, const mrpt::viz::Scene& scene) -> mrpt::img::CImage
          {
            mrpt::img::CImage img;
            self.render_RGB(scene, img);
            return img;
          },
          py::arg("scene"), "Render scene to an RGB CImage")
      .def(
          "render_depth",
          [](CFBORender& self, const mrpt::viz::Scene& scene)
          {
            mrpt::math::CMatrixFloat depth;
            self.render_depth(scene, depth);
            return toNumpyDepth(depth);
          },
          py::arg("scene"), "Render scene to a depth map: a float32 NumPy array (rows, cols)")
      .def(
          "render_RGBD",
          [](CFBORender& self, const mrpt::viz::Scene& scene)
          {
            mrpt::img::CImage rgb;
            mrpt::math::CMatrixFloat depth;
            self.render_RGBD(scene, rgb, depth);
            return std::make_tuple(rgb, toNumpyDepth(depth));
          },
          py::arg("scene"), "Render scene to a tuple (RGB CImage, depth float32 NumPy array)");
}
