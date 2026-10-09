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
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

// MRPT headers
#include <mrpt/img/CImage.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/img/TColor.h>
#include <mrpt/img/TPixelCoord.h>
#include <mrpt/img/TStereoCamera.h>
#include <mrpt/img/color_maps.h>

#include <stdexcept>
#include <vector>

namespace py = pybind11;
using namespace mrpt::img;
using namespace pybind11::literals;  // Enables the _a suffix

namespace
{
// Builds a CImage from a NumPy uint8 array of shape (H, W) or (H, W, C), with
// C = 1 (gray), 3 (RGB) or 4 (RGBA).
CImage::Ptr imageFromNumpy(const py::array_t<uint8_t>& array)
{
  if (array.ndim() != 2 && array.ndim() != 3)
  {
    throw std::invalid_argument("Expected an array of shape (H, W) or (H, W, C)");
  }
  const py::ssize_t h = array.shape(0);
  const py::ssize_t w = array.shape(1);
  const py::ssize_t nCh = array.ndim() == 3 ? array.shape(2) : 1;
  if (nCh != 1 && nCh != 3 && nCh != 4)
  {
    throw std::invalid_argument("The number of channels must be 1, 3 or 4");
  }
  auto img = CImage::Create();
  img->resize(static_cast<int32_t>(w), static_cast<int32_t>(h), static_cast<TImageChannels>(nCh));
  for (py::ssize_t y = 0; y < h; y++)
  {
    for (py::ssize_t x = 0; x < w; x++)
    {
      for (py::ssize_t c = 0; c < nCh; c++)
      {
        const uint8_t v = array.ndim() == 3 ? array.at(y, x, c) : array.at(y, x);
        img->at<uint8_t>(static_cast<int>(x), static_cast<int>(y), static_cast<int8_t>(c)) = v;
      }
    }
  }
  return img;
}
}  // namespace

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
  py::class_<TColor>(m, "TColor", "An RGBA color, 8 bits per channel.")
      .def(
          py::init<uint8_t, uint8_t, uint8_t, uint8_t>(), "r"_a, "g"_a, "b"_a, "alpha"_a = 255,
          "Builds a color from its components (0-255).")
      .def(
          py::init(
              [](const std::vector<uint8_t>& v)
              {
                if (v.size() < 3)
                {
                  throw std::invalid_argument("List must have 3 or 4 elements");
                }
                return TColor(v[0], v[1], v[2], v.size() > 3 ? v[3] : 255);
              }),
          "Builds a color from a list [r, g, b] or [r, g, b, a] (0-255).")
      .def_readwrite("R", &TColor::R)
      .def_readwrite("G", &TColor::G)
      .def_readwrite("B", &TColor::B)
      .def_readwrite("A", &TColor::A)
      .def(py::self == py::self)
      .def(py::self != py::self)
      .def(
          "__repr__",
          [](const TColor& c)
          {
            return "TColor(" + std::to_string(c.R) + ", " + std::to_string(c.G) + ", " +
                   std::to_string(c.B) + ", " + std::to_string(c.A) + ")";
          });

  // TColorf: RGBA color with float components in [0,1]
  py::class_<TColorf>(m, "TColorf", "An RGBA color - floats in the range [0,1].")
      .def(py::init<>(), "Default constructor.")
      .def(
          py::init<float, float, float, float>(), "r"_a, "g"_a, "b"_a, "alpha"_a = 1.0f,
          "Builds a color from its components (0-1).")
      .def(py::init<const TColor&>(), "color"_a, "Builds a float color from an 8-bit TColor.")
      .def_readwrite("R", &TColorf::R)
      .def_readwrite("G", &TColorf::G)
      .def_readwrite("B", &TColorf::B)
      .def_readwrite("A", &TColorf::A)
      .def("asTColor", &TColorf::asTColor, "Converts to a TColor with uint8 components")
      .def(
          "__repr__",
          [](const TColorf& c)
          {
            return "TColorf(" + std::to_string(c.R) + ", " + std::to_string(c.G) + ", " +
                   std::to_string(c.B) + ", " + std::to_string(c.A) + ")";
          });
  py::implicitly_convertible<TColor, TColorf>();

  // 3. TPixelCoord and TPixelCoordf
  py::class_<TPixelCoord>(m, "TPixelCoord", "Integer pixel coordinates (x, y).")
      .def(py::init<int, int>(), "Builds a pixel coordinate from (x, y).")
      .def_readwrite("x", &TPixelCoord::x)
      .def_readwrite("y", &TPixelCoord::y);
  py::class_<TPixelCoordf>(m, "TPixelCoordf", "Sub-pixel coordinates (x, y), as floats.")
      .def(py::init<float, float>(), "Builds a pixel coordinate from (x, y).")
      .def_readwrite("x", &TPixelCoordf::x)
      .def_readwrite("y", &TPixelCoordf::y);

  // 4. CImage (The most important part)
  py::enum_<PixelDepth>(m, "PixelDepth", "Bit depth of each image channel")
      .value("D8U", PixelDepth::D8U)
      .value("D16U", PixelDepth::D16U);

  py::class_<CImage, std::shared_ptr<CImage>>(
      m, "CImage", "A class for storing images as grayscale, RGB, or RGBA bitmaps.")
      .def(py::init<>(), "Default constructor: an empty image.")
      // Pythonic Constructor from NumPy array
      .def(
          py::init(&imageFromNumpy), "array"_a,
          "Builds an image from a NumPy uint8 array of shape (height, width) or (height, width, "
          "channels), with 1, 3 or 4 channels.")
      .def(
          "resize",
          [](CImage& img, int32_t width, int32_t height, int channels, PixelDepth depth)
          { img.resize(width, height, static_cast<TImageChannels>(channels), depth); },
          py::arg("width"), py::arg("height"), py::arg("channels") = 3,
          py::arg("depth") = PixelDepth::D8U,
          "Changes the image size and number of channels (1, 3 or 4), erasing its contents "
          "(it does not scale them).")
      .def(
          "as_numpy",
          [](const py::object& self_obj)
          {
            // We take py::object self_obj instead of CImage& self so we can
            // use it as the 'base' for the numpy array to prevent crashes.
            auto& self = self_obj.cast<CImage&>();

            const auto h = static_cast<py::ssize_t>(self.getHeight());
            const auto w = static_cast<py::ssize_t>(self.getWidth());
            const auto nCh = static_cast<py::ssize_t>(self.channels());
            const bool is16bit = self.getPixelDepth() == PixelDepth::D16U;
            const py::ssize_t chBytes = is16bit ? 2 : 1;

            // Strides in bytes: rows, pixels, channels
            const std::vector<py::ssize_t> shape = {h, w, nCh};
            const std::vector<py::ssize_t> strides = {
                static_cast<py::ssize_t>(self.getRowStride()), nCh * chBytes, chBytes};

            // self_obj is the array "base": it keeps the CImage alive.
            if (is16bit)
            {
              return py::array(
                  py::dtype::of<uint16_t>(), shape, strides, self.ptrLine<uint16_t>(0), self_obj);
            }
            return py::array(
                py::dtype::of<uint8_t>(), shape, strides, self.ptrLine<uint8_t>(0), self_obj);
          },
          "Returns a zero-copy NumPy view of the image, of shape (height, width, channels) and "
          "dtype uint8 or uint16.")  // Drawing methods (from CCanvas)
      .def(
          "drawCircle", &CImage::drawCircle, "center"_a, "radius"_a, "color"_a, "width"_a = 1,
          "Draws a circle of a given radius.")
      .def(
          "textOut", &CImage::textOut, "p"_a, "str"_a, "color"_a,
          "Renders 2D text using bitmap fonts.")
      // Load/save
      .def(
          "loadFromFile",
          [](CImage& self, const std::string& filename) { return self.loadFromFile(filename); },
          "filename"_a, "Load image from file. Returns True on success.")
      .def(
          "saveToFile",
          [](const CImage& self, const std::string& filename, int jpeg_quality)
          { return self.saveToFile(filename, jpeg_quality); },
          "filename"_a, "jpeg_quality"_a = 95, "Save image to file. Returns True on success.")
      // Geometry queries
      .def("getWidth", &CImage::getWidth, "Image width in pixels")
      .def("getHeight", &CImage::getHeight, "Image height in pixels")
      .def("isColor", &CImage::isColor, "True if the image has 3 or 4 channels (RGB or RGBA)")
      // Static factory from numpy array (complement to as_numpy)
      .def_static(
          "from_numpy", &imageFromNumpy, "array"_a,
          "Creates a CImage from a NumPy uint8 array of shape (H, W) or (H, W, C), with 1, 3 or 4 "
          "channels (the data is copied).");

  // 5. TCamera
  py::class_<TCamera>(
      m, "TCamera",
      "Intrinsic parameters for a pinhole or fisheye camera model, along with the associated lens "
      "distortion model.")
      .def(py::init<>(), "Default constructor: all intrinsic parameters set to zero.")
      .def_readwrite("ncols", &TCamera::ncols)
      .def_readwrite("nrows", &TCamera::nrows)
      .def_readwrite("distortion", &TCamera::distortion)
      .def_property(
          "dist",
          [](const TCamera& c) { return std::vector<double>(c.dist.begin(), c.dist.end()); },
          [](TCamera& c, const std::vector<double>& v)
          { std::copy(v.begin(), v.end(), c.dist.begin()); })
      .def_property(
          "fx", [](const TCamera& c) { return c.fx(); }, [](TCamera& c, double v) { c.fx(v); })
      .def_property(
          "fy", [](const TCamera& c) { return c.fy(); }, [](TCamera& c, double v) { c.fy(v); })
      .def_property(
          "cx", [](const TCamera& c) { return c.cx(); }, [](TCamera& c, double v) { c.cx(v); })
      .def_property(
          "cy", [](const TCamera& c) { return c.cy(); }, [](TCamera& c, double v) { c.cy(v); })
      .def(
          "intrinsicParams",
          [](const TCamera& c)
          {
            const auto& K = c.intrinsicParams;
            return std::vector<std::vector<double>>{
                {K(0, 0), K(0, 1), K(0, 2)},
                {K(1, 0), K(1, 1), K(1, 2)},
                {K(2, 0), K(2, 1), K(2, 2)}
            };
          },
          "Return intrinsic matrix as 3x3 list of lists (use np.array() to convert).")
      .def("__repr__", [](const TCamera& c) { return c.dumpAsText(); });

  // 6. TStereoCamera
  py::class_<TStereoCamera, mrpt::serialization::CSerializable, std::shared_ptr<TStereoCamera>>(
      m, "TStereoCamera", "Structure to hold the parameters of a pinhole stereo camera model.")
      .def(py::init<>(), "Default constructor.")
      .def_readwrite("leftCamera", &TStereoCamera::leftCamera)
      .def_readwrite("rightCamera", &TStereoCamera::rightCamera)
      .def_readwrite("rightCameraPose", &TStereoCamera::rightCameraPose)
      .def("__repr__", [](const TStereoCamera& c) { return c.dumpAsText(); });

  // 7. Colormap helpers
  py::enum_<TColormap>(m, "TColormap")
      .value("cmNONE", cmNONE)
      .value("cmGRAYSCALE", cmGRAYSCALE)
      .value("cmJET", cmJET)
      .value("cmHOT", cmHOT)
      .export_values();
  m.def(
      "colormap", &colormap, "color_map"_a, "color_index"_a,
      "Maps a value in [0,1] to a TColorf using the given colormap");
}