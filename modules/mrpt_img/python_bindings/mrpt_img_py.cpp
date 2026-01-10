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
#include <mrpt/img/CImage.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/img/TColor.h>
#include <mrpt/img/TPixelCoord.h>
#include <mrpt/img/TStereoCamera.h>
#include <mrpt/img/color_maps.h>

namespace py = pybind11;
using namespace mrpt::img;
using namespace pybind11::literals;  // Enables the _a suffix

PYBIND11_MODULE(_bindings, m)
{
  m.doc() = "Python bindings for mrpt-img";

  // 1. Enums
  py::enum_<DistortionModel>(m, "DistortionModel")
      .value("none", DistortionModel::none)
      .value("plumb_bob", DistortionModel::plumb_bob)
      .value("kannala_brandt", DistortionModel::kannala_brandt)
      .export_values();

  // 2. TColor and TColorf
  py::class_<TColor>(m, "TColor")
      .def(py::init<uint8_t, uint8_t, uint8_t, uint8_t>(), "r"_a, "g"_a, "b"_a, "alpha"_a = 255)
      .def(py::init(
          [](const std::vector<uint8_t>& v)
          {
            if (v.size() < 3)
            {
              throw std::invalid_argument("List must have 3 or 4 elements");
            }
            return TColor(v[0], v[1], v[2], v.size() > 3 ? v[3] : 255);
          }))
      .def_readwrite("R", &TColor::R)
      .def_readwrite("G", &TColor::G)
      .def_readwrite("B", &TColor::B)
      .def_readwrite("A", &TColor::A);

  // 3. TPixelCoord and TPixelCoordf
  py::class_<TPixelCoord>(m, "TPixelCoord")
      .def(py::init<int, int>())
      .def_readwrite("x", &TPixelCoord::x)
      .def_readwrite("y", &TPixelCoord::y);
  py::class_<TPixelCoordf>(m, "TPixelCoordf")
      .def(py::init<float, float>())
      .def_readwrite("x", &TPixelCoordf::x)
      .def_readwrite("y", &TPixelCoordf::y);

  // 4. CImage (The most important part)
  py::class_<CImage, std::shared_ptr<CImage>>(m, "CImage")
      .def(py::init<>())
      // Pythonic Constructor from NumPy array
      .def(py::init(
          [](const py::array_t<uint8_t>& array)
          {
            auto r = array.unchecked<3>();
            auto img = CImage::Create();
            img->resize(
                static_cast<int32_t>(r.shape(1)), static_cast<int32_t>(r.shape(0)),
                r.shape(2) == 3 ? CH_RGB : CH_GRAY);

            // Copy data from numpy to MRPT
            for (int y = 0; y < r.shape(0); y++)
            {
              for (int x = 0; x < r.shape(1); x++)
              {
                for (int c = 0; c < r.shape(2); c++)
                {
                  img->at<uint8_t>(x, y, static_cast<int8_t>(c)) = r(y, x, c);
                }
              }
            }
            return img;
          }))
      .def("resize", &CImage::resize)
      .def(
          "as_numpy",
          [](const py::object& self_obj)
          {
            // We take py::object self_obj instead of CImage& self so we can
            // use it as the 'base' for the numpy array to prevent crashes.
            auto& self = self_obj.cast<CImage&>();

            size_t h = self.getHeight();
            size_t w = self.getWidth();
            size_t channels = self.isColor() ? 3 : 1;

            // Strides in BYTES:
            // A row is (width * channels) bytes long.
            // A pixel is (channels) bytes long.
            // A single channel is 1 byte long.
            std::vector<size_t> shape = {h, w, channels};
            std::vector<size_t> strides = {
                self.getRowStride(),  // Row stride in bytes
                channels,             // Pixel stride
                1                     // Channel stride
            };

            return py::array_t<uint8_t>(
                shape, strides, self.ptrLine<uint8_t>(0),
                self_obj  // This is the "base" - keeps the CImage alive!
            );
          },
          "Returns a Zero-Copy NumPy view of the image data.")  // Drawing methods (from CCanvas)
      .def("drawCircle", &CImage::drawCircle, "center"_a, "radius"_a, "color"_a, "width"_a = 1)
      .def("textOut", &CImage::textOut, "p"_a, "str"_a, "color"_a);

  // 5. TCamera
  py::class_<TCamera>(m, "TCamera")
      .def(py::init<>())
      .def_readwrite("ncols", &TCamera::ncols)
      .def_readwrite("nrows", &TCamera::nrows)
      .def_readwrite("distortion", &TCamera::distortion)
      .def_property(
          "dist",
          [](const TCamera& c) { return std::vector<double>(c.dist.begin(), c.dist.end()); },
          [](TCamera& c, const std::vector<double>& v)
          { std::copy(v.begin(), v.end(), c.dist.begin()); })
      .def("intrinsicParams", [](const TCamera& c) { return c.intrinsicParams; });

  // 6. Colormap helpers
  m.def("colormap", &colormap, "color_map"_a, "color_index"_a);
}