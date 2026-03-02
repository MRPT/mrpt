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
#include <mrpt/img/TCamera.h>
#include <mrpt/vision/CImagePyramid.h>
#include <mrpt/vision/CUndistortMap.h>
#include <mrpt/vision/TKeyPoint.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_MODULE(_bindings, m)
{
  m.doc() = "Python bindings for mrpt::vision — image processing and computer vision";

  // -------------------------------------------------------------------------
  // TKeyPoint — 2D image keypoint (integer pixel coordinates)
  // -------------------------------------------------------------------------
  py::class_<mrpt::vision::TKeyPoint>(m, "TKeyPoint")
      .def(py::init<>())
      .def(py::init<int, int>(), "x"_a, "y"_a)
      .def_readwrite("pt", &mrpt::vision::TKeyPoint::pt, "Pixel coordinates (TPixelCoord)")
      .def_readwrite(
          "response", &mrpt::vision::TKeyPoint::response, "Goodness/saliency of the feature")
      .def_readwrite(
          "octave", &mrpt::vision::TKeyPoint::octave, "Image pyramid octave (0=original)")
      .def(
          "__repr__",
          [](const mrpt::vision::TKeyPoint& kp)
          {
            return "TKeyPoint(x=" + std::to_string(kp.pt.x) + ", y=" + std::to_string(kp.pt.y) +
                   ", response=" + std::to_string(kp.response) + ")";
          });

  // -------------------------------------------------------------------------
  // TKeyPointf — 2D image keypoint (float pixel coordinates)
  // -------------------------------------------------------------------------
  py::class_<mrpt::vision::TKeyPointf>(m, "TKeyPointf")
      .def(py::init<>())
      .def(py::init<float, float>(), "x"_a, "y"_a)
      .def_readwrite("pt", &mrpt::vision::TKeyPointf::pt, "Pixel coordinates (TPixelCoordf)")
      .def_readwrite(
          "response", &mrpt::vision::TKeyPointf::response, "Goodness/saliency of the feature")
      .def_readwrite(
          "octave", &mrpt::vision::TKeyPointf::octave, "Image pyramid octave (0=original)")
      .def(
          "__repr__",
          [](const mrpt::vision::TKeyPointf& kp)
          {
            return "TKeyPointf(x=" + std::to_string(kp.pt.x) + ", y=" + std::to_string(kp.pt.y) +
                   ", response=" + std::to_string(kp.response) + ")";
          });

  // -------------------------------------------------------------------------
  // CUndistortMap — pre-computed undistortion map for fast image rectification
  // -------------------------------------------------------------------------
  py::class_<mrpt::vision::CUndistortMap>(m, "CUndistortMap")
      .def(py::init<>())
      .def(
          "setFromCamParams", &mrpt::vision::CUndistortMap::setFromCamParams, "camera"_a,
          "Initialize the undistortion map from TCamera parameters")
      .def(
          "undistort",
          py::overload_cast<const mrpt::img::CImage&, mrpt::img::CImage&>(
              &mrpt::vision::CUndistortMap::undistort, py::const_),
          "in_img"_a, "out_img"_a, "Undistort an image and write to out_img")
      .def(
          "undistort_inplace",
          py::overload_cast<mrpt::img::CImage&>(
              &mrpt::vision::CUndistortMap::undistort, py::const_),
          "in_out_img"_a, "Undistort an image in-place")
      .def(
          "isSet", &mrpt::vision::CUndistortMap::isSet,
          "True if the map has been initialized with setFromCamParams()")
      .def(
          "__repr__", [](const mrpt::vision::CUndistortMap& m)
          { return "CUndistortMap(isSet=" + std::string(m.isSet() ? "True" : "False") + ")"; });

  // -------------------------------------------------------------------------
  // CImagePyramid — Gaussian image pyramid for multi-scale processing
  // -------------------------------------------------------------------------
  py::class_<mrpt::vision::CImagePyramid>(m, "CImagePyramid")
      .def(py::init<>())
      .def(
          "buildFromImage",
          [](mrpt::vision::CImagePyramid& p, const mrpt::img::CImage& img, size_t nOctaves,
             bool smooth_imgs, bool convert_grayscale)
          { p.buildFromImage(img, nOctaves, smooth_imgs, convert_grayscale); },
          "img"_a, "nOctaves"_a = 4, "smooth_imgs"_a = true, "convert_grayscale"_a = true,
          "Build an N-octave pyramid from the given image")
      .def(
          "buildFromImageAndComputeIntegral",
          [](mrpt::vision::CImagePyramid& p, const mrpt::img::CImage& img, size_t nOctaves,
             bool smooth_imgs, bool convert_grayscale)
          { p.buildFromImageAndComputeIntegral(img, nOctaves, smooth_imgs, convert_grayscale); },
          "img"_a, "nOctaves"_a = 4, "smooth_imgs"_a = true, "convert_grayscale"_a = true)
      .def(
          "getOctave",
          [](const mrpt::vision::CImagePyramid& p, size_t octave) -> const mrpt::img::CImage&
          { return p.images[octave]; },
          "octave"_a, py::return_value_policy::reference_internal,
          "Get the CImage for a given pyramid octave (0=original)")
      .def(
          "octaveCount", [](const mrpt::vision::CImagePyramid& p) { return p.images.size(); },
          "Return the number of octaves in the pyramid")
      .def(
          "__repr__", [](const mrpt::vision::CImagePyramid& p)
          { return "CImagePyramid(" + std::to_string(p.images.size()) + " octaves)"; });
}
