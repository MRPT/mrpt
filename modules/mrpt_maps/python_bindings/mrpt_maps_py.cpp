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

#include <mrpt/maps/CBeacon.h>
#include <mrpt/maps/CBeaconMap.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/maps/CHeightGridMap2D.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/COccupancyGridMap3D.h>
#include <mrpt/maps/COctoMap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/CVoxelMap.h>
#include <mrpt/maps/CVoxelMapRGB.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/customizable_obs_viz.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/viz/CSetOfObjects.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <limits>
#include <optional>

namespace py = pybind11;
using namespace pybind11::literals;

namespace
{
template <typename T>
struct type_tag
{
  using type = T;
};
}  // namespace

PYBIND11_MODULE(_bindings, m)
{
  m.doc() = "Python bindings for mrpt::maps — metric map representations";

  // -------------------------------------------------------------------------
  // CMetricMap — abstract base for all metric maps
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::maps::CMetricMap, mrpt::serialization::CSerializable,
      std::shared_ptr<mrpt::maps::CMetricMap>>(
      m, "CMetricMap", "Declares a virtual base class for all metric maps storage classes.")
      .def("clear", &mrpt::maps::CMetricMap::clear, "Erase all the contents of the map.")
      .def(
          "isEmpty", &mrpt::maps::CMetricMap::isEmpty,
          "Returns true if the map is empty/no observation has been inserted.")
      .def(
          "insertObservation",
          [](mrpt::maps::CMetricMap& mp, const mrpt::obs::CObservation& obs,
             const mrpt::poses::CPose3D* robotPose)
          {
            if (robotPose)
            {
              return mp.insertObservation(
                  obs, std::optional<const mrpt::poses::CPose3D>(*robotPose));
            }
            return mp.insertObservation(obs, std::optional<const mrpt::poses::CPose3D>());
          },
          "obs"_a, "robotPose"_a = nullptr,
          "Insert an observation into the map. Returns true if the map was updated.")
      .def(
          "insertObs",
          [](mrpt::maps::CMetricMap& mp, const mrpt::obs::CSensoryFrame& sf,
             const std::optional<mrpt::poses::CPose3D>& robotPose)
          {
            if (robotPose)
            {
              return sf.insertObservationsInto(
                  mp, std::optional<const mrpt::poses::CPose3D>(*robotPose));
            }
            return sf.insertObservationsInto(mp);
          },
          "sf"_a, "robotPose"_a = std::nullopt,
          "Inserts all the observations of a CSensoryFrame. Returns true if any was inserted.")
      .def(
          "loadFromSimpleMap", &mrpt::maps::CMetricMap::loadFromSimpleMap, "simpleMap"_a,
          "Clears the map and builds it from all keyframes of a CSimpleMap")
      .def(
          "computeObservationLikelihood",
          [](const mrpt::maps::CMetricMap& mp, const mrpt::obs::CObservation& obs,
             const mrpt::poses::CPose3D& takenFrom)
          { return mp.computeObservationLikelihood(obs, takenFrom); },
          "obs"_a, "takenFrom"_a, "Log-likelihood of an observation taken from a given robot pose")
      .def(
          "computeObservationsLikelihood",
          [](mrpt::maps::CMetricMap& mp, const mrpt::obs::CSensoryFrame& sf,
             const mrpt::poses::CPose3D& takenFrom)
          { return mp.computeObservationsLikelihood(sf, takenFrom); },
          "sf"_a, "takenFrom"_a,
          "Log-likelihood of all observations in a CSensoryFrame taken from a given pose")
      .def(
          "canComputeObservationLikelihood",
          &mrpt::maps::CMetricMap::canComputeObservationLikelihood, "obs"_a,
          "Returns true if this map is able to compute a sensible likelihood function for this "
          "observation (i.e. an occupancy grid map cannot with an image).")
      .def(
          "getVisualization",
          [](const mrpt::maps::CMetricMap& mp) { return mp.getVisualization(); },
          "Returns a 3D representation of the map as a CSetOfObjects")
      .def(
          "getVisualizationInto", &mrpt::maps::CMetricMap::getVisualizationInto, "outObj"_a,
          "Appends a 3D representation of the map to a CSetOfObjects")
      .def(
          "boundingBox",
          [](const mrpt::maps::CMetricMap& mp)
          {
            const auto bb = mp.boundingBox();
            return mrpt::math::TBoundingBox(
                mrpt::math::TPoint3D(bb.min.x, bb.min.y, bb.min.z),
                mrpt::math::TPoint3D(bb.max.x, bb.max.y, bb.max.z),
                mrpt::math::TBoundingBox::CTOR_FLAGS::AllowUnordered);
          },
          "Bounding box of the map contents")
      .def_readwrite("genericMapParams", &mrpt::maps::CMetricMap::genericMapParams)
      .def(
          "saveMetricMapRepresentationToFile",
          &mrpt::maps::CMetricMap::saveMetricMapRepresentationToFile, "filNamePrefix"_a,
          "Saves the map in a format suitable for inspection (e.g. images or text)")
      .def("__str__", &mrpt::maps::CMetricMap::asString)
      .def(
          "GetRuntimeClass", &mrpt::maps::CMetricMap::GetRuntimeClass,
          "Returns information about the class of an object in runtime.");

  // -------------------------------------------------------------------------
  // CPointsMap — abstract point cloud map
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::maps::CPointsMap, mrpt::maps::CMetricMap, std::shared_ptr<mrpt::maps::CPointsMap>>(
      m, "CPointsMap",
      "A cloud of points in 2D or 3D, which can be built from a sequence of laser scans or other "
      "sensors.")
      .def("size", &mrpt::maps::CPointsMap::size, "Returns the number of points.")
      .def("__len__", &mrpt::maps::CPointsMap::size)
      .def(
          "isEmpty", &mrpt::maps::CPointsMap::isEmpty,
          "Returns true if the map is empty/no observation has been inserted.")
      .def(
          "reserve", &mrpt::maps::CPointsMap::reserve,
          "Reserves memory for a given number of points, without changing the map size.")
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
          "x"_a, "y"_a, "z"_a = 0.0f,
          "Provides a way to insert (append) individual points into the map: the missing fields of "
          "child classes (color, weight, etc) are left to their default values.")
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
      .def(
          "save2D_to_text_file", &mrpt::maps::CPointsMap::save2D_to_text_file,
          "Save to a text file. Each line will contain \"X Y\" point coordinates.")
      .def(
          "save3D_to_text_file", &mrpt::maps::CPointsMap::save3D_to_text_file,
          "Save to a text file. Each line will contain \"X Y Z\" point coordinates.")
      .def(
          "load2D_from_text_file", &mrpt::maps::CPointsMap::load2D_from_text_file,
          "Load from a text file. Each line should contain an \"X Y\" coordinate pair, separated "
          "by whitespaces.")
      .def(
          "load3D_from_text_file", &mrpt::maps::CPointsMap::load3D_from_text_file,
          "Load from a text file. Each line should contain an \"X Y Z\" coordinate tuple, "
          "separated by whitespaces.")
      .def(
          "__repr__", [](const mrpt::maps::CPointsMap& mp)
          { return "CPointsMap(" + std::to_string(mp.size()) + " points)"; });

  // -------------------------------------------------------------------------
  // CSimplePointsMap — concrete XYZ point cloud
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::maps::CSimplePointsMap, mrpt::maps::CPointsMap,
      std::shared_ptr<mrpt::maps::CSimplePointsMap>>(
      m, "CSimplePointsMap",
      "A cloud of points in 2D or 3D, which can be built from a sequence of laser scans.")
      .def(py::init<>(), "Default constructor.")
      .def(
          "__repr__", [](const mrpt::maps::CSimplePointsMap& mp)
          { return "CSimplePointsMap(" + std::to_string(mp.size()) + " points)"; });

  // -------------------------------------------------------------------------
  // CGenericPointsMap — XYZ point cloud + arbitrary string-keyed data channels
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::maps::CGenericPointsMap, mrpt::maps::CPointsMap,
      std::shared_ptr<mrpt::maps::CGenericPointsMap>>(
      m, "CGenericPointsMap",
      "A map of 3D points (X,Y,Z) plus any number of custom, string-keyed per-point data channels.")
      .def(py::init<>(), "Default constructor.")
      // Register custom per-point data channels:
      .def(
          "registerField_float", &mrpt::maps::CGenericPointsMap::registerField_float, "fieldName"_a,
          "Register a new per-point data channel of type float32")
      .def(
          "registerField_double", &mrpt::maps::CGenericPointsMap::registerField_double,
          "fieldName"_a, "Register a new per-point data channel of type float64")
      .def(
          "registerField_uint16", &mrpt::maps::CGenericPointsMap::registerField_uint16,
          "fieldName"_a, "Register a new per-point data channel of type uint16")
      .def(
          "registerField_uint8", &mrpt::maps::CGenericPointsMap::registerField_uint8, "fieldName"_a,
          "Register a new per-point data channel of type uint8")
      .def(
          "registerField_uint32", &mrpt::maps::CGenericPointsMap::registerField_uint32,
          "fieldName"_a, "Register a new per-point data channel of type uint32 (New in MRPT 3.0.0)")
      .def(
          "unregisterField", &mrpt::maps::CGenericPointsMap::unregisterField, "fieldName"_a,
          "Removes a data channel; returns True if it existed")
      .def(
          "hasPointField", &mrpt::maps::CGenericPointsMap::hasPointField, "fieldName"_a,
          "Returns true if the map has a data channel with the given name.")
      .def(
          "resize", &mrpt::maps::CGenericPointsMap::resize, "newLength"_a,
          "Resizes all point buffers so they can hold the given number of points: newly created "
          "points are set to default values, and old contents are not changed.")
      // Field name enumeration:
      .def(
          "getPointFieldNames_float", &mrpt::maps::CGenericPointsMap::getPointFieldNames_float,
          "Get list of all float channel names.")
      .def(
          "getPointFieldNames_double", &mrpt::maps::CGenericPointsMap::getPointFieldNames_double,
          "Get list of all double channel names.")
      .def(
          "getPointFieldNames_uint16", &mrpt::maps::CGenericPointsMap::getPointFieldNames_uint16,
          "Get list of all uint16_t channel names.")
      .def(
          "getPointFieldNames_uint8", &mrpt::maps::CGenericPointsMap::getPointFieldNames_uint8,
          "Get list of all uint8_t channel names.")
      .def(
          "getPointFieldNames_uint32", &mrpt::maps::CGenericPointsMap::getPointFieldNames_uint32,
          "List all uint32 channel names (New in MRPT 3.0.0)")
      // Per-point field getters:
      .def(
          "getPointField_float", &mrpt::maps::CGenericPointsMap::getPointField_float, "index"_a,
          "fieldName"_a,
          "Read the value of a float channel for a given point. Returns 0 if field does not exist.")
      .def(
          "getPointField_double", &mrpt::maps::CGenericPointsMap::getPointField_double, "index"_a,
          "fieldName"_a,
          "Read the value of a double channel for a given point. Returns 0 if field does not "
          "exist.")
      .def(
          "getPointField_uint16", &mrpt::maps::CGenericPointsMap::getPointField_uint16, "index"_a,
          "fieldName"_a,
          "Read the value of a uint16_t channel for a given point. Returns 0 if field does not "
          "exist.")
      .def(
          "getPointField_uint8", &mrpt::maps::CGenericPointsMap::getPointField_uint8, "index"_a,
          "fieldName"_a,
          "Read the value of a uint8_t channel for a given point. Returns 0 if field does not "
          "exist.")
      .def(
          "getPointField_uint32", &mrpt::maps::CGenericPointsMap::getPointField_uint32, "index"_a,
          "fieldName"_a, "Read a uint32 channel value (New in MRPT 3.0.0)")
      // Per-point field setters:
      .def(
          "setPointField_float", &mrpt::maps::CGenericPointsMap::setPointField_float, "index"_a,
          "fieldName"_a, "value"_a, "Sets the value of a float channel for a given point.")
      .def(
          "setPointField_double", &mrpt::maps::CGenericPointsMap::setPointField_double, "index"_a,
          "fieldName"_a, "value"_a, "Sets the value of a double channel for a given point.")
      .def(
          "setPointField_uint16", &mrpt::maps::CGenericPointsMap::setPointField_uint16, "index"_a,
          "fieldName"_a, "value"_a, "Sets the value of a uint16_t channel for a given point.")
      .def(
          "setPointField_uint8", &mrpt::maps::CGenericPointsMap::setPointField_uint8, "index"_a,
          "fieldName"_a, "value"_a, "Sets the value of a uint8_t channel for a given point.")
      .def(
          "setPointField_uint32", &mrpt::maps::CGenericPointsMap::setPointField_uint32, "index"_a,
          "fieldName"_a, "value"_a, "Set a uint32 channel value (New in MRPT 3.0.0)")
      .def(
          "__repr__", [](const mrpt::maps::CGenericPointsMap& mp)
          { return "CGenericPointsMap(" + std::to_string(mp.size()) + " points)"; });

  // -------------------------------------------------------------------------
  // COccupancyGridMap2D — probabilistic 2D occupancy grid
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::maps::COccupancyGridMap2D, mrpt::maps::CMetricMap,
      std::shared_ptr<mrpt::maps::COccupancyGridMap2D>>(
      m, "COccupancyGridMap2D",
      "A 2D occupancy grid map: each cell holds its probability of being occupied.")
      .def(
          py::init<float, float, float, float, float>(), "xMin"_a = -10.0f, "xMax"_a = 10.0f,
          "yMin"_a = -10.0f, "yMax"_a = 10.0f, "resolution"_a = 0.10f, "Constructor.")
      // Grid metadata
      .def(
          "getSizeX", &mrpt::maps::COccupancyGridMap2D::getSizeX,
          "Returns the horizontal size of grid map in cells count.")
      .def(
          "getSizeY", &mrpt::maps::COccupancyGridMap2D::getSizeY,
          "Returns the vertical size of grid map in cells count.")
      .def(
          "getXMin", &mrpt::maps::COccupancyGridMap2D::getXMin,
          "Returns the \"x\" coordinate of left side of grid map.")
      .def(
          "getXMax", &mrpt::maps::COccupancyGridMap2D::getXMax,
          "Returns the \"x\" coordinate of right side of grid map.")
      .def(
          "getYMin", &mrpt::maps::COccupancyGridMap2D::getYMin,
          "Returns the \"y\" coordinate of top side of grid map.")
      .def(
          "getYMax", &mrpt::maps::COccupancyGridMap2D::getYMax,
          "Returns the \"y\" coordinate of bottom side of grid map.")
      .def(
          "getResolution", &mrpt::maps::COccupancyGridMap2D::getResolution,
          "Returns the resolution of the grid map.")
      .def(
          "isEmpty", &mrpt::maps::COccupancyGridMap2D::isEmpty,
          "Returns true upon map construction or after calling clear(), the return changes to "
          "false upon successful insertObservation() or any other method to load data in the map.")
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
          "x"_a,
          "Transform a coordinate value into a cell index. Uses floor() to correctly handle "
          "negative coordinates near zero.")
      .def(
          "y2idx", [](const mrpt::maps::COccupancyGridMap2D& g, float y) { return g.y2idx(y); },
          "y"_a, "Transforms a y coordinate into a cell index.")
      .def(
          "idx2x", &mrpt::maps::COccupancyGridMap2D::idx2x,
          "Transform a cell index into a coordinate value (center of the cell)")
      .def(
          "idx2y", &mrpt::maps::COccupancyGridMap2D::idx2y,
          "Transforms a cell index into a y coordinate (center of the cell).")
      // File I/O
      .def(
          "saveAsBitmapFile", &mrpt::maps::COccupancyGridMap2D::saveAsBitmapFile,
          "Saves the grid map as an image file; the format is given by the file extension.")
      .def(
          "loadFromBitmapFile",
          [](mrpt::maps::COccupancyGridMap2D& g, const std::string& file, float resolution)
          { return g.loadFromBitmapFile(file, resolution); },
          "file"_a, "resolution"_a,
          "Loads the grid map from an image file, given its resolution and origin.")
      .def(
          "loadFromROSMapServerYAML", &mrpt::maps::COccupancyGridMap2D::loadFromROSMapServerYAML,
          "yamlFilePath"_a, "Load a ROS map_server YAML + PNG/PGM file pair")
      // NumPy integration — export grid as HxW float32 array
      .def(
          "getAsNumpy",
          [](const mrpt::maps::COccupancyGridMap2D& g)
          {
            const size_t sx = g.getSizeX();
            const size_t sy = g.getSizeY();
            py::array_t<float> arr({sy, sx});
            auto buf = arr.mutable_unchecked<2>();
            for (size_t row = 0; row < sy; row++)
            {
              for (size_t col = 0; col < sx; col++)
              {
                buf(row, col) = g.getCell(static_cast<int>(col), static_cast<int>(row));
              }
            }
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

  // -------------------------------------------------------------------------
  // CMultiMetricMap: a container of heterogeneous metric maps
  // -------------------------------------------------------------------------
  using MMap = mrpt::maps::CMultiMetricMap;
  py::class_<MMap, mrpt::maps::CMetricMap, std::shared_ptr<MMap>>(
      m, "CMultiMetricMap", "A set of metric maps of any type, updated and queried together.")
      .def(py::init<>(), "Default ctor: empty list of maps.")
      .def(
          py::init<const mrpt::maps::TSetOfMetricMapInitializers&>(), "initializers"_a,
          "Creates the maps described by a TSetOfMetricMapInitializers")
      .def(
          "setListOfMaps", &MMap::setListOfMaps, "initializers"_a,
          "Replaces all maps with the ones described by a TSetOfMetricMapInitializers")
      .def("size", &MMap::size, "Number of child maps.")
      .def("__len__", &MMap::size)
      .def("push_back", &MMap::push_back, "map"_a, "Appends a new child map to the list.")
      .def("clearMaps", &MMap::clearMaps, "Removes all maps (clear() only empties them)")
      .def(
          "mapByIndex", [](MMap& mm, size_t i) { return mm.mapByIndex(i); }, "index"_a,
          "Gets the i-th map.")
      .def(
          "__getitem__",
          [](MMap& mm, size_t i)
          {
            if (i >= mm.size())
            {
              throw py::index_error();
            }
            return mm.mapByIndex(i);
          })
      .def(
          "__setitem__",
          [](MMap& mm, size_t i, const mrpt::maps::CMetricMap::Ptr& newMap)
          {
            if (i >= mm.size())
            {
              throw py::index_error();
            }
            *(mm.begin() + static_cast<std::ptrdiff_t>(i)) = newMap;
          },
          "Replaces the i-th map")
      .def(
          "__iter__", [](MMap& mm) { return py::make_iterator(mm.begin(), mm.end()); },
          py::keep_alive<0, 1>())
      .def_property_readonly(
          "maps",
          [](MMap& mm) { return std::vector<mrpt::maps::CMetricMap::Ptr>(mm.begin(), mm.end()); },
          "A list with all the maps (to replace one, use map[i] = newMap)")
      .def(
          "__repr__",
          [](const MMap& mm) { return "CMultiMetricMap(" + std::to_string(mm.size()) + " maps)"; });

  // -------------------------------------------------------------------------
  // Voxel maps (Bonxai-based sparse voxel grids)
  // -------------------------------------------------------------------------
  const auto bindVoxelMap = [&](auto clsTag, const char* name, const char* doc)
  {
    using T = typename decltype(clsTag)::type;
    py::class_<T, mrpt::maps::CMetricMap, std::shared_ptr<T>>(m, name, doc)
        .def(
            py::init<double, uint8_t, uint8_t>(), "resolution"_a = 0.05, "inner_bits"_a = 2,
            "leaf_bits"_a = 3, "Creates an empty map with the given voxel size (meters).")
        .def(
            "updateVoxel", &T::updateVoxel, "x"_a, "y"_a, "z"_a, "occupied"_a,
            "Updates one voxel with an occupied or free observation")
        .def(
            "getPointOccupancy",
            [](const T& vm, double x, double y, double z) -> std::optional<double>
            {
              double p = 0;
              if (!vm.getPointOccupancy(x, y, z, p))
              {
                return std::nullopt;
              }
              return p;
            },
            "x"_a, "y"_a, "z"_a,
            "Occupancy probability [0,1] of the voxel at a point, or None if not observed")
        .def(
            "insertPointCloudAsRays",
            [](T& vm, const mrpt::maps::CPointsMap& pts, const mrpt::math::TPoint3D& sensorPt)
            { vm.insertPointCloudAsRays(pts, sensorPt); },
            "points"_a, "sensorPt"_a,
            "Inserts a point cloud, marking free space along the rays from sensorPt")
        .def(
            "insertPointCloudAsEndPoints",
            [](T& vm, const mrpt::maps::CPointsMap& pts, const mrpt::math::TPoint3D& sensorPt)
            { vm.insertPointCloudAsEndPoints(pts, sensorPt); },
            "points"_a, "sensorPt"_a, "Inserts a point cloud updating only the end points")
        .def(
            "getOccupiedVoxels", [](T& vm) { return vm.getOccupiedVoxels(); },
            "Returns the centers of all occupied voxels as a CSimplePointsMap")
        .def(
            "__repr__",
            [name](const T& vm) {
              return std::string(name) + "(resolution=" + std::to_string(vm.grid().resolution) +
                     ")";
            });
  };
  bindVoxelMap(
      type_tag<mrpt::maps::CVoxelMap>{}, "CVoxelMap",
      "A sparse 3D occupancy voxel map, with log-odds occupancy per voxel.");
  bindVoxelMap(
      type_tag<mrpt::maps::CVoxelMapRGB>{}, "CVoxelMapRGB",
      "A sparse 3D occupancy voxel map, with log-odds occupancy and an RGB color per voxel.");

  // -------------------------------------------------------------------------
  // COccupancyGridMap3D: dense 3D occupancy grid
  // -------------------------------------------------------------------------
  using Grid3D = mrpt::maps::COccupancyGridMap3D;
  py::class_<Grid3D, mrpt::maps::CMetricMap, std::shared_ptr<Grid3D>>(
      m, "COccupancyGridMap3D",
      "A 3D occupancy grid map with a regular, even distribution of voxels.")
      .def(
          py::init<const mrpt::math::TPoint3D&, const mrpt::math::TPoint3D&, float>(),
          "corner_min"_a = mrpt::math::TPoint3D(-5.0, -5.0, -5.0),
          "corner_max"_a = mrpt::math::TPoint3D(5.0, 5.0, 5.0), "resolution"_a = 0.25f,
          "Constructor.")
      .def("fill", &Grid3D::fill, "default_value"_a = 0.5f, "Sets all voxels to a freeness value")
      .def(
          "getSizeX", [](const Grid3D& g) { return g.m_grid.getSizeX(); }, "Number of voxels in X")
      .def(
          "getSizeY", [](const Grid3D& g) { return g.m_grid.getSizeY(); }, "Number of voxels in Y")
      .def(
          "getSizeZ", [](const Grid3D& g) { return g.m_grid.getSizeZ(); }, "Number of voxels in Z")
      .def(
          "getResolution", [](const Grid3D& g) { return g.m_grid.getResolutionXY(); },
          "Voxel size (meters)")
      .def(
          "getCellFreeness", &Grid3D::getCellFreeness, "cx"_a, "cy"_a, "cz"_a,
          "Freeness probability [0,1] of a voxel by index (1 = free)")
      .def(
          "setCellFreeness", &Grid3D::setCellFreeness, "cx"_a, "cy"_a, "cz"_a, "value"_a,
          "Sets the freeness probability [0,1] of a voxel by index")
      .def(
          "getFreenessByPos", &Grid3D::getFreenessByPos, "x"_a, "y"_a, "z"_a,
          "Freeness probability [0,1] at a metric position (1 = free)")
      .def(
          "setFreenessByPos", &Grid3D::setFreenessByPos, "x"_a, "y"_a, "z"_a, "value"_a,
          "Sets the freeness probability [0,1] at a metric position")
      .def(
          "__repr__",
          [](const Grid3D& g)
          {
            return "COccupancyGridMap3D(size=" + std::to_string(g.m_grid.getSizeX()) + "x" +
                   std::to_string(g.m_grid.getSizeY()) + "x" + std::to_string(g.m_grid.getSizeZ()) +
                   ")";
          });

  // -------------------------------------------------------------------------
  // CHeightGridMap2D: 2.5D elevation map
  // -------------------------------------------------------------------------
  using HMap = mrpt::maps::CHeightGridMap2D;
  py::class_<HMap, mrpt::maps::CMetricMap, std::shared_ptr<HMap>>(
      m, "CHeightGridMap2D",
      "Digital Elevation Model (DEM), a mesh or grid representation of a surface which keeps the "
      "estimated height for each (x,y) location.")
      .def(
          py::init(
              [](double xMin, double xMax, double yMin, double yMax, double resolution) {
                return std::make_shared<HMap>(
                    HMap::mrSimpleAverage, xMin, xMax, yMin, yMax, resolution);
              }),
          "xMin"_a = -2.0, "xMax"_a = 2.0, "yMin"_a = -2.0, "yMax"_a = 2.0, "resolution"_a = 0.1,
          "Creates a height map with the given limits and resolution.")
      .def(
          "getSizeX", [](const HMap& h) { return h.dem_get_size_x(); },
          "Returns the horizontal size of grid map in cells count.")
      .def(
          "getSizeY", [](const HMap& h) { return h.dem_get_size_y(); },
          "Returns the vertical size of grid map in cells count.")
      .def(
          "getResolution", [](const HMap& h) { return h.dem_get_resolution(); },
          "Returns the resolution of the grid map.")
      .def(
          "getXMin", [](const HMap& h) { return h.getXMin(); },
          "Returns the \"x\" coordinate of left side of grid map.")
      .def(
          "getYMin", [](const HMap& h) { return h.getYMin(); },
          "Returns the \"y\" coordinate of top side of grid map.")
      .def(
          "countObservedCells", &HMap::countObservedCells,
          "Return the number of cells with at least one height data inserted.")
      .def(
          "insertIndividualPoint",
          [](HMap& h, double x, double y, double z) { return h.insertIndividualPoint(x, y, z); },
          "x"_a, "y"_a, "z"_a, "Inserts one (x,y,z) point. Returns False if out of the map.")
      .def(
          "getHeight",
          [](const HMap& h, double x, double y) -> std::optional<double>
          {
            double z = 0;
            if (!h.dem_get_z(x, y, z))
            {
              return std::nullopt;
            }
            return z;
          },
          "x"_a, "y"_a, "Height at a metric position, or None if not observed")
      .def(
          "getAsNumpy",
          [](const HMap& h)
          {
            const size_t sx = h.dem_get_size_x();
            const size_t sy = h.dem_get_size_y();
            py::array_t<double> arr(std::vector<py::ssize_t>{py::ssize_t(sy), py::ssize_t(sx)});
            auto buf = arr.mutable_unchecked<2>();
            for (size_t cy = 0; cy < sy; cy++)
            {
              for (size_t cx = 0; cx < sx; cx++)
              {
                double z = 0;
                buf(cy, cx) =
                    h.dem_get_z_by_cell(cx, cy, z) ? z : std::numeric_limits<double>::quiet_NaN();
              }
            }
            return arr;
          },
          "Returns the heights as an HxW float64 array (NaN for unobserved cells)");

  // -------------------------------------------------------------------------
  // CBeaconMap: map of range-only beacons
  // -------------------------------------------------------------------------
  py::class_<
      mrpt::maps::CBeacon, mrpt::serialization::CSerializable,
      std::shared_ptr<mrpt::maps::CBeacon>>(
      m, "CBeacon",
      "The class for storing individual \"beacon landmarks\" under a variety of 3D position PDF "
      "distributions.")
      .def(py::init<>(), "Default constructor.")
      .def_readwrite("m_ID", &mrpt::maps::CBeacon::m_ID, "Beacon ID")
      .def(
          "getMean",
          [](const mrpt::maps::CBeacon& b)
          {
            mrpt::poses::CPoint3D p;
            b.getMean(p);
            return p;
          },
          "Mean position of the beacon");

  py::class_<
      mrpt::maps::CBeaconMap, mrpt::maps::CMetricMap, std::shared_ptr<mrpt::maps::CBeaconMap>>(
      m, "CBeaconMap",
      "A class for storing a map of 3D probabilistic beacons, using a Montecarlo, Gaussian, or Sum "
      "of Gaussians (SOG) representation (for range-only SLAM).")
      .def(py::init<>(), "Constructor.")
      .def("size", &mrpt::maps::CBeaconMap::size, "Returns the stored landmarks count.")
      .def("__len__", &mrpt::maps::CBeaconMap::size)
      .def(
          "push_back", &mrpt::maps::CBeaconMap::push_back, "beacon"_a,
          "Appends a beacon to the map.")
      .def(
          "__getitem__",
          [](mrpt::maps::CBeaconMap& bm, size_t i) -> mrpt::maps::CBeacon&
          {
            if (i >= bm.size())
            {
              throw py::index_error();
            }
            return bm.get(i);
          },
          py::return_value_policy::reference_internal);

  // -------------------------------------------------------------------------
  // COctoMap: OctoMap-based probabilistic 3D occupancy map
  // -------------------------------------------------------------------------
  py::class_<mrpt::maps::COctoMap, mrpt::maps::CMetricMap, std::shared_ptr<mrpt::maps::COctoMap>>(
      m, "COctoMap",
      "A three-dimensional probabilistic occupancy grid, implemented as an octo-tree with the "
      "\"octomap\" C++ library.")
      .def(py::init<double>(), "resolution"_a = 0.10, "Default constructor.")
      .def(
          "getResolution", &mrpt::maps::COctoMap::getResolution,
          "Returns the size of the octomap leaf voxels.")
      .def("size", &mrpt::maps::COctoMap::size, "Number of octree nodes")
      .def(
          "updateVoxel", &mrpt::maps::COctoMap::updateVoxel, "x"_a, "y"_a, "z"_a, "occupied"_a,
          "Updates one voxel with an occupied or free observation")
      .def(
          "isPointWithinOctoMap", &mrpt::maps::COctoMap::isPointWithinOctoMap, "x"_a, "y"_a, "z"_a,
          "Check whether the given point lies within the volume covered by the octomap (that is, "
          "whether it is \"mapped\")")
      .def(
          "getPointOccupancy",
          [](const mrpt::maps::COctoMap& om, float x, float y, float z)
          { return om.getPointOccupancy(x, y, z); },
          "x"_a, "y"_a, "z"_a,
          "Occupancy probability [0,1] at a point, or None if the point is not in the octree")
      .def(
          "insertPointCloud", &mrpt::maps::COctoMap::insertPointCloud, "points"_a, "sensor_x"_a,
          "sensor_y"_a, "sensor_z"_a, "Inserts a point cloud as rays from the sensor position")
      .def(
          "getMetricMin",
          [](const mrpt::maps::COctoMap& om)
          {
            double x = 0;
            double y = 0;
            double z = 0;
            om.getMetricMin(x, y, z);
            return mrpt::math::TPoint3D(x, y, z);
          },
          "Minimum value of the bounding box of all known space in x, y, z.")
      .def(
          "getMetricMax",
          [](const mrpt::maps::COctoMap& om)
          {
            double x = 0;
            double y = 0;
            double z = 0;
            om.getMetricMax(x, y, z);
            return mrpt::math::TPoint3D(x, y, z);
          },
          "Maximum value of the bounding box of all known space in x, y, z.");

  // -------------------------------------------------------------------------
  // CObservationPointCloud (in mrpt::obs, but part of the mrpt_maps library)
  // -------------------------------------------------------------------------
  using ObsPC = mrpt::obs::CObservationPointCloud;
  py::class_<ObsPC, mrpt::obs::CObservation, std::shared_ptr<ObsPC>>(
      m, "CObservationPointCloud",
      "An observation from any sensor that can be summarized as a pointcloud.")
      .def(py::init<>(), "Default constructor.")
      .def(
          py::init<const mrpt::obs::CObservation3DRangeScan&>(), "scan"_a,
          "Builds a point cloud observation from the 3D points of a depth scan")
      .def_readwrite("pointcloud", &ObsPC::pointcloud, "The point cloud (a CPointsMap)")
      .def_readwrite("sensorPose", &ObsPC::sensorPose)
      .def(
          "isExternallyStored", &ObsPC::isExternallyStored,
          "Returns true if the point cloud is stored in an external file.")
      .def(
          "getExternalStorageFile", &ObsPC::getExternalStorageFile,
          "Returns the external file name of the point cloud, if any.")
      .def(
          "__repr__",
          [](const ObsPC& o)
          {
            return "CObservationPointCloud(label='" + o.sensorLabel +
                   "', points=" + std::to_string(o.pointcloud ? o.pointcloud->size() : 0) + ")";
          });

  // -------------------------------------------------------------------------
  // Visualization of observations
  // -------------------------------------------------------------------------
  using RecolorParams = mrpt::obs::PointCloudRecoloringParameters;
  py::class_<RecolorParams>(
      m, "PointCloudRecoloringParameters",
      "Parameters for recolorize3Dpc(), or part of VisualizationParameters if using obs_to_viz()")
      .def(py::init<>(), "Default constructor.")
      .def_readwrite("colorizeByField", &RecolorParams::colorizeByField)
      .def_readwrite("invertColorMapping", &RecolorParams::invertColorMapping)
      .def_readwrite("colorMap", &RecolorParams::colorMap)
      .def_readwrite("colorMapMinCoord", &RecolorParams::colorMapMinCoord)
      .def_readwrite("colorMapMaxCoord", &RecolorParams::colorMapMaxCoord)
      .def_readwrite("outlierRejectionPercentile", &RecolorParams::outlierRejectionPercentile);

  using VizParams = mrpt::obs::VisualizationParameters;
  py::class_<VizParams>(
      m, "VisualizationParameters",
      "Here we can customize the way observations will be rendered as 3D objects in obs_to_viz(), "
      "obs3Dscan_to_viz(), etc.")
      .def(py::init<>(), "Default constructor.")
      .def_readwrite("coloring", &VizParams::coloring)
      .def_readwrite("showAxis", &VizParams::showAxis)
      .def_readwrite("axisTickFrequency", &VizParams::axisTickFrequency)
      .def_readwrite("axisLimits", &VizParams::axisLimits)
      .def_readwrite("axisTickTextSize", &VizParams::axisTickTextSize)
      .def_readwrite("colorFromRGBimage", &VizParams::colorFromRGBimage)
      .def_readwrite("pointSize", &VizParams::pointSize)
      .def_readwrite("drawSensorPose", &VizParams::drawSensorPose)
      .def_readwrite("sensorPoseScale", &VizParams::sensorPoseScale)
      .def_readwrite("onlyPointsWithColor", &VizParams::onlyPointsWithColor)
      .def_readwrite("showSurfaceIn2Dscans", &VizParams::showSurfaceIn2Dscans)
      .def_readwrite("showPointsIn2Dscans", &VizParams::showPointsIn2Dscans)
      .def_readwrite("surface2DscansColor", &VizParams::surface2DscansColor)
      .def_readwrite("points2DscansColor", &VizParams::points2DscansColor);

  m.def(
      "obs_to_viz",
      [](const mrpt::obs::CObservation::Ptr& obs, const VizParams& p,
         mrpt::viz::CSetOfObjects::Ptr out)
      {
        if (!out)
        {
          out = mrpt::viz::CSetOfObjects::Create();
        }
        mrpt::obs::obs_to_viz(obs, p, *out);
        return out;
      },
      "obs"_a, "params"_a = VizParams(), "out"_a = nullptr,
      "Renders an observation into a CSetOfObjects (a new one if out is None), and returns it");
  m.def(
      "obs_to_viz",
      [](const mrpt::obs::CSensoryFrame& sf, const VizParams& p, mrpt::viz::CSetOfObjects::Ptr out)
      {
        if (!out)
        {
          out = mrpt::viz::CSetOfObjects::Create();
        }
        mrpt::obs::obs_to_viz(sf, p, *out);
        return out;
      },
      "sf"_a, "params"_a = VizParams(), "out"_a = nullptr,
      "Renders all observations of a CSensoryFrame into a CSetOfObjects (a new one if out is "
      "None), and returns it");
}
