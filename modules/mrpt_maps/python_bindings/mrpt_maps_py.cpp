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

#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CSerializable.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_MODULE(_bindings, m)
{
  m.doc() = "Python bindings for mrpt::maps — metric map representations";

  // -------------------------------------------------------------------------
  // CMetricMap — abstract base for all metric maps
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::maps::CMetricMap, mrpt::serialization::CSerializable,
      std::shared_ptr<mrpt::maps::CMetricMap>>(m, "CMetricMap")
      .def("clear", &mrpt::maps::CMetricMap::clear)
      .def("isEmpty", &mrpt::maps::CMetricMap::isEmpty)
      .def(
          "insertObservation",
          [](mrpt::maps::CMetricMap& mp, const mrpt::obs::CObservation& obs,
             const mrpt::poses::CPose3D* robotPose)
          {
            if (robotPose)
              return mp.insertObservation(
                  obs, std::optional<const mrpt::poses::CPose3D>(*robotPose));
            else
              return mp.insertObservation(obs, std::optional<const mrpt::poses::CPose3D>());
          },
          "obs"_a, "robotPose"_a = nullptr,
          "Insert an observation into the map. Returns true if the map was updated.")
      .def("GetRuntimeClass", &mrpt::maps::CMetricMap::GetRuntimeClass);

  // -------------------------------------------------------------------------
  // CPointsMap — abstract point cloud map
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::maps::CPointsMap, mrpt::maps::CMetricMap, std::shared_ptr<mrpt::maps::CPointsMap>>(
      m, "CPointsMap")
      .def("size", &mrpt::maps::CPointsMap::size)
      .def("__len__", &mrpt::maps::CPointsMap::size)
      .def("isEmpty", &mrpt::maps::CPointsMap::isEmpty)
      .def("reserve", &mrpt::maps::CPointsMap::reserve)
      .def(
          "getPoint",
          [](const mrpt::maps::CPointsMap& mp, size_t i)
          {
            float x, y, z;
            mp.getPoint(i, x, y, z);
            return py::make_tuple(x, y, z);
          },
          "i"_a, "Returns (x, y, z) tuple for point i")
      .def(
          "insertPoint",
          [](mrpt::maps::CPointsMap& mp, float x, float y, float z) { mp.insertPoint(x, y, z); },
          "x"_a, "y"_a, "z"_a = 0.0f)
      // NumPy integration — get all points as Nx3 float32 array
      .def(
          "getPointsAsNumpy",
          [](const mrpt::maps::CPointsMap& mp)
          {
            const size_t n = mp.size();
            py::array_t<float> arr({n, size_t(3)});
            auto buf = arr.mutable_unchecked<2>();
            for (size_t i = 0; i < n; i++)
            {
              float x, y, z;
              mp.getPoint(i, x, y, z);
              buf(i, 0) = x;
              buf(i, 1) = y;
              buf(i, 2) = z;
            }
            return arr;
          },
          "Returns all points as an Nx3 float32 numpy array")
      // Load all points from Nx3 numpy array
      .def(
          "setPointsFromNumpy",
          [](mrpt::maps::CPointsMap& mp, const py::array_t<float>& arr)
          {
            auto r = arr.unchecked<2>();
            mp.clear();
            mp.reserve(r.shape(0));
            for (py::ssize_t i = 0; i < r.shape(0); i++) mp.insertPoint(r(i, 0), r(i, 1), r(i, 2));
          },
          "arr"_a, "Load an Nx3 float32 numpy array into this point cloud")
      .def("save2D_to_text_file", &mrpt::maps::CPointsMap::save2D_to_text_file)
      .def("save3D_to_text_file", &mrpt::maps::CPointsMap::save3D_to_text_file)
      .def("load2D_from_text_file", &mrpt::maps::CPointsMap::load2D_from_text_file)
      .def("load3D_from_text_file", &mrpt::maps::CPointsMap::load3D_from_text_file)
      .def(
          "__repr__", [](const mrpt::maps::CPointsMap& mp)
          { return "CPointsMap(" + std::to_string(mp.size()) + " points)"; });

  // -------------------------------------------------------------------------
  // CSimplePointsMap — concrete XYZ point cloud
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::maps::CSimplePointsMap, mrpt::maps::CPointsMap,
      std::shared_ptr<mrpt::maps::CSimplePointsMap>>(m, "CSimplePointsMap")
      .def(py::init<>())
      .def(
          "__repr__", [](const mrpt::maps::CSimplePointsMap& mp)
          { return "CSimplePointsMap(" + std::to_string(mp.size()) + " points)"; });

  // -------------------------------------------------------------------------
  // COccupancyGridMap2D — probabilistic 2D occupancy grid
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::maps::COccupancyGridMap2D, mrpt::maps::CMetricMap,
      std::shared_ptr<mrpt::maps::COccupancyGridMap2D>>(m, "COccupancyGridMap2D")
      .def(
          py::init<float, float, float, float, float>(), "xMin"_a = -10.0f, "xMax"_a = 10.0f,
          "yMin"_a = -10.0f, "yMax"_a = 10.0f, "resolution"_a = 0.10f)
      // Grid metadata
      .def("getSizeX", &mrpt::maps::COccupancyGridMap2D::getSizeX)
      .def("getSizeY", &mrpt::maps::COccupancyGridMap2D::getSizeY)
      .def("getXMin", &mrpt::maps::COccupancyGridMap2D::getXMin)
      .def("getXMax", &mrpt::maps::COccupancyGridMap2D::getXMax)
      .def("getYMin", &mrpt::maps::COccupancyGridMap2D::getYMin)
      .def("getYMax", &mrpt::maps::COccupancyGridMap2D::getYMax)
      .def("getResolution", &mrpt::maps::COccupancyGridMap2D::getResolution)
      .def("isEmpty", &mrpt::maps::COccupancyGridMap2D::isEmpty)
      // Cell access by grid index
      .def(
          "getCell",
          [](const mrpt::maps::COccupancyGridMap2D& g, int x, int y) { return g.getCell(x, y); },
          "x"_a, "y"_a, "Get occupancy probability [0,1] at cell (x,y)")
      .def(
          "setCell",
          [](mrpt::maps::COccupancyGridMap2D& g, int x, int y, float val) { g.setCell(x, y, val); },
          "x"_a, "y"_a, "value"_a, "Set occupancy probability [0,1] at cell (x,y)")
      // Cell access by metric coordinates
      .def(
          "getPos",
          [](const mrpt::maps::COccupancyGridMap2D& g, float x, float y) { return g.getPos(x, y); },
          "x"_a, "y"_a, "Get occupancy probability at metric position (x,y)")
      .def(
          "setPos",
          [](mrpt::maps::COccupancyGridMap2D& g, float x, float y, float val)
          { g.setPos(x, y, val); },
          "x"_a, "y"_a, "value"_a, "Set occupancy probability at metric position (x,y)")
      // Index ↔ metric conversion
      .def(
          "x2idx", [](const mrpt::maps::COccupancyGridMap2D& g, float x) { return g.x2idx(x); },
          "x"_a)
      .def(
          "y2idx", [](const mrpt::maps::COccupancyGridMap2D& g, float y) { return g.y2idx(y); },
          "y"_a)
      .def("idx2x", &mrpt::maps::COccupancyGridMap2D::idx2x)
      .def("idx2y", &mrpt::maps::COccupancyGridMap2D::idx2y)
      // File I/O
      .def("saveAsBitmapFile", &mrpt::maps::COccupancyGridMap2D::saveAsBitmapFile)
      .def(
          "loadFromBitmapFile",
          [](mrpt::maps::COccupancyGridMap2D& g, const std::string& file, double resolution,
             float xCentralPixel, float yCentralPixel)
          { return g.loadFromBitmapFile(file, resolution, xCentralPixel, yCentralPixel); },
          "file"_a, "resolution"_a, "xCentralPixel"_a = -1.0f, "yCentralPixel"_a = -1.0f)
      .def(
          "loadFromROSMapServerYAML", &mrpt::maps::COccupancyGridMap2D::loadFromROSMapServerYAML,
          "yamlFilePath"_a, "Load a ROS map_server YAML + PNG/PGM file pair")
      // NumPy integration — export grid as HxW float32 array
      .def(
          "getAsNumpy",
          [](const mrpt::maps::COccupancyGridMap2D& g)
          {
            const size_t sx = g.getSizeX(), sy = g.getSizeY();
            py::array_t<float> arr({sy, sx});
            auto buf = arr.mutable_unchecked<2>();
            for (size_t row = 0; row < sy; row++)
              for (size_t col = 0; col < sx; col++)
                buf(row, col) = g.getCell(static_cast<int>(col), static_cast<int>(row));
            return arr;
          },
          "Returns the occupancy grid as an HxW float32 numpy array (0=occupied, 1=free)")
      .def(
          "__repr__",
          [](const mrpt::maps::COccupancyGridMap2D& g)
          {
            return "COccupancyGridMap2D(size=" + std::to_string(g.getSizeX()) + "x" +
                   std::to_string(g.getSizeY()) + ", res=" + std::to_string(g.getResolution()) +
                   ")";
          });
}
