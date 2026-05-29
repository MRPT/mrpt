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

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <mrpt/img/CImage.h>
#include <mrpt/math/CMatrixF.h>
#include <mrpt/opengl/CFBORender.h>
#include <mrpt/viz/CCamera.h>
#include <mrpt/viz/Scene.h>

namespace py = pybind11;
using namespace mrpt::opengl;
using namespace pybind11::literals;

PYBIND11_MODULE(_bindings, m)
{
  // CFBORender::Parameters
  py::class_<CFBORender::Parameters>(m, "CFBORenderParameters")
      .def(
          py::init<unsigned int, unsigned int>(), py::arg("width") = 800,
          py::arg("height") = 600)
      .def_readwrite("width", &CFBORender::Parameters::width)
      .def_readwrite("height", &CFBORender::Parameters::height)
      .def_readwrite("raw_depth", &CFBORender::Parameters::raw_depth)
      .def_readwrite("create_EGL_context", &CFBORender::Parameters::create_EGL_context)
      .def_readwrite("deviceIndexToUse", &CFBORender::Parameters::deviceIndexToUse)
      .def_readwrite("contextMajorVersion", &CFBORender::Parameters::contextMajorVersion)
      .def_readwrite("contextMinorVersion", &CFBORender::Parameters::contextMinorVersion)
      .def_readwrite("contextDebug", &CFBORender::Parameters::contextDebug);

  // CFBORender
  py::class_<CFBORender>(m, "CFBORender")
      .def(
          py::init<unsigned int, unsigned int>(), py::arg("width") = 800,
          py::arg("height") = 600)
      .def("setCamera", &CFBORender::setCamera, py::arg("camera"))
      .def("clearCameraOverride", &CFBORender::clearCameraOverride)
      .def("hasCameraOverride", &CFBORender::hasCameraOverride)
      .def("width", &CFBORender::width)
      .def("height", &CFBORender::height)
      .def("invalidateCompiledScene", &CFBORender::invalidateCompiledScene)
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
          [](CFBORender& self, const mrpt::viz::Scene& scene) -> mrpt::math::CMatrixFloat
          {
            mrpt::math::CMatrixFloat depth;
            self.render_depth(scene, depth);
            return depth;
          },
          py::arg("scene"), "Render scene to a depth map (float matrix)")
      .def(
          "render_RGBD",
          [](CFBORender& self, const mrpt::viz::Scene& scene)
          {
            mrpt::img::CImage rgb;
            mrpt::math::CMatrixFloat depth;
            self.render_RGBD(scene, rgb, depth);
            return py::make_tuple(rgb, depth);
          },
          py::arg("scene"), "Render scene to (RGB CImage, depth CMatrixFloat)");
}
