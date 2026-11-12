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
#include <mrpt/obs/CObservation2DRangeScan.h>
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

#include <algorithm>
#include <cstdio>
#include <functional>
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

// The Python class already registered for T, to add more methods to it:
template <class T>
py::class_<T, std::shared_ptr<T>> registeredClass()
{
  return py::reinterpret_borrow<py::class_<T, std::shared_ptr<T>>>(py::type::of<T>());
}

// Results of a kNN or radius search, as (indices, points, squared distances) NumPy arrays.
template <int DIM, class POINT>
py::tuple nnResultsAsNumpy(
    const std::vector<POINT>& pts,
    const std::vector<float>& distsSqr,
    const std::vector<uint64_t>& indices)
{
  const auto n = static_cast<py::ssize_t>(pts.size());
  py::array_t<uint64_t> outIdx(std::vector<py::ssize_t>{n});
  py::array_t<float> outPts(std::vector<py::ssize_t>{n, DIM});
  py::array_t<float> outDists(std::vector<py::ssize_t>{n});
  auto bufIdx = outIdx.mutable_unchecked<1>();
  auto bufPts = outPts.mutable_unchecked<2>();
  auto bufDists = outDists.mutable_unchecked<1>();
  for (py::ssize_t i = 0; i < n; i++)
  {
    bufIdx(i) = indices[i];
    bufDists(i) = distsSqr[i];
    for (int k = 0; k < DIM; k++)
    {
      bufPts(i, k) = pts[i][k];
    }
  }
  return py::make_tuple(outIdx, outPts, outDists);
}

// A NumPy copy of the first n values of a point field, read from its buffer if
// available, or with the per-point getter otherwise.
template <typename T>
py::array_t<T> fieldToNumpy(
    const mrpt::aligned_std_vector<T>* buf, size_t n, const std::function<T(size_t)>& getter)
{
  py::array_t<T> arr(std::vector<py::ssize_t>{static_cast<py::ssize_t>(n)});
  T* out = arr.mutable_data();
  if (buf != nullptr && buf->size() >= n)
  {
    std::copy_n(buf->data(), n, out);
  }
  else
  {
    for (size_t i = 0; i < n; i++)
    {
      out[i] = getter(i);
    }
  }
  return arr;
}

// Registers a field of type T in a CGenericPointsMap and copies its values
// from a 1D array of n elements.
template <typename T>
void registerAndFillField(
    mrpt::maps::CGenericPointsMap& mp,
    const std::string& name,
    const py::array& values,
    bool (mrpt::maps::CGenericPointsMap::*registerFn)(const std::string&),
    mrpt::aligned_std_vector<T>* (mrpt::maps::CGenericPointsMap::*bufferFn)(const std::string&))
{
  (mp.*registerFn)(name);
  mp.resize(mp.size());  // sizes the new field
  auto* buf = (mp.*bufferFn)(name);
  const auto arr = py::array_t<T, py::array::c_style | py::array::forcecast>::ensure(values);
  if (buf == nullptr || !arr || static_cast<size_t>(arr.size()) != buf->size())
  {
    throw py::value_error("cannot set the values of field '" + name + "'");
  }
  std::copy_n(arr.data(), buf->size(), buf->data());
}
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
          py::return_value_policy::reference,
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
      // NumPy integration: get or set all points as an Nx3 float32 array
      .def(
          "getPointsAsNumpy",
          [](const mrpt::maps::CPointsMap& mp)
          {
            const auto& xs = mp.getPointsBufferRef_x();
            const auto& ys = mp.getPointsBufferRef_y();
            const auto& zs = mp.getPointsBufferRef_z();
            const auto n = static_cast<py::ssize_t>(mp.size());
            py::array_t<float> arr(std::vector<py::ssize_t>{n, 3});
            auto buf = arr.mutable_unchecked<2>();
            for (py::ssize_t i = 0; i < n; i++)
            {
              buf(i, 0) = xs[i];
              buf(i, 1) = ys[i];
              buf(i, 2) = zs[i];
            }
            return arr;
          },
          "Returns all points as an Nx3 float32 numpy array")
      .def(
          "setPointsFromNumpy",
          [](mrpt::maps::CPointsMap& mp,
             const py::array_t<float, py::array::c_style | py::array::forcecast>& arr)
          {
            if (arr.ndim() != 2 || arr.shape(1) != 3)
            {
              throw py::value_error("The array must have shape (N, 3)");
            }
            const auto r = arr.unchecked<2>();
            mp.setSize(static_cast<size_t>(r.shape(0)));
            for (py::ssize_t i = 0; i < r.shape(0); i++)
            {
              mp.setPointFast(static_cast<size_t>(i), r(i, 0), r(i, 1), r(i, 2));
            }
            mp.mark_as_modified();
          },
          "arr"_a,
          "Replaces all the points with those of an Nx3 array (other point fields are reset to "
          "their default values).")
      .def(
          "getPointFieldsAsNumpy",
          [](const mrpt::maps::CPointsMap& mp)
          {
            const size_t n = mp.size();
            py::dict out;
            for (const auto& name : mp.getPointFieldNames_float())
            {
              out[py::str(name)] = fieldToNumpy<float>(
                  mp.getPointsBufferRef_float_field(name), n,
                  [&](size_t i) { return mp.getPointField_float(i, name); });
            }
            for (const auto& name : mp.getPointFieldNames_double())
            {
              out[py::str(name)] = fieldToNumpy<double>(
                  mp.getPointsBufferRef_double_field(name), n,
                  [&](size_t i) { return mp.getPointField_double(i, name); });
            }
            for (const auto& name : mp.getPointFieldNames_uint16())
            {
              out[py::str(name)] = fieldToNumpy<uint16_t>(
                  mp.getPointsBufferRef_uint16_field(name), n,
                  [&](size_t i) { return mp.getPointField_uint16(i, name); });
            }
            for (const auto& name : mp.getPointFieldNames_uint8())
            {
              out[py::str(name)] = fieldToNumpy<uint8_t>(
                  mp.getPointsBufferRef_uint8_field(name), n,
                  [&](size_t i) { return mp.getPointField_uint8(i, name); });
            }
            for (const auto& name : mp.getPointFieldNames_uint32())
            {
              out[py::str(name)] = fieldToNumpy<uint32_t>(
                  mp.getPointsBufferRef_uint32_field(name), n,
                  [&](size_t i) { return mp.getPointField_uint32(i, name); });
            }
            return out;
          },
          "Returns a dict with a 1D NumPy array (a copy) for x, y, z and every other point field, "
          "with its dtype: float32, float64, uint16, uint8 or uint32.")
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
          "loadFromPlyFile",
          [](mrpt::maps::CPointsMap& mp, const std::string& filename)
          { return mp.loadFromPlyFile(filename); },
          "filename"_a,
          "Loads the points from a PLY file. Returns False on error: see getLoadPLYErrorString().")
      .def(
          "getLoadPLYErrorString", &mrpt::maps::CPointsMap::getLoadPLYErrorString,
          "Description of the error of the last failed loadFromPlyFile().")
      .def(
          "saveToPlyFile",
          [](const mrpt::maps::CPointsMap& mp, const std::string& filename, bool save_in_binary,
             const std::vector<std::string>& file_comments)
          {
            // The exporter does not handle files that cannot be created:
            std::FILE* f = std::fopen(filename.c_str(), "w");
            if (f == nullptr)
            {
              return false;
            }
            std::fclose(f);
            return mp.saveToPlyFile(filename, save_in_binary, file_comments);
          },
          "filename"_a, "save_in_binary"_a = false, "file_comments"_a = std::vector<std::string>(),
          "Saves the points to a PLY file (text or binary). Returns False if the file cannot be "
          "created, or on other errors: see getSavePLYErrorString().")
      .def(
          "getSavePLYErrorString", &mrpt::maps::CPointsMap::getSavePLYErrorString,
          "Description of the error of the last failed saveToPlyFile().")
      .def(
          "loadFromKittiVelodyneFile", &mrpt::maps::CPointsMap::loadFromKittiVelodyneFile,
          "filename"_a,
          "Loads a KITTI Velodyne scan (binary XYZI, gzip-compressed if the name ends in \".gz\"), "
          "with the intensity in the \"intensity\" field if the map supports it. Returns True on "
          "success.")
      .def(
          "saveToKittiVelodyneFile", &mrpt::maps::CPointsMap::saveToKittiVelodyneFile, "filename"_a,
          "Saves the points and their \"intensity\" field in the binary XYZI format of the KITTI "
          "dataset, always compressed: use a file name ending in \".gz\" to load it back. Returns "
          "True on success.")
      // Nearest-neighbor queries (KD-tree)
      .def(
          "nn_single_search",
          [](const mrpt::maps::CPointsMap& mp, const mrpt::math::TPoint3D& query) -> py::object
          {
            mrpt::math::TPoint3Df result;
            float distSqr = 0;
            uint64_t idx = 0;
            if (!mp.nn_single_search(mrpt::math::TPoint3Df(query), result, distSqr, idx))
            {
              return py::none();
            }
            return py::make_tuple(idx, mrpt::math::TPoint3D(result), distSqr);
          },
          "query"_a,
          "Closest point to a 3D query point (KD-tree search). Returns (index, point, squared "
          "distance), or None if none was found.")
      .def(
          "nn_single_search",
          [](const mrpt::maps::CPointsMap& mp, const mrpt::math::TPoint2D& query) -> py::object
          {
            mrpt::math::TPoint2Df result;
            float distSqr = 0;
            uint64_t idx = 0;
            if (!mp.nn_single_search(mrpt::math::TPoint2Df(query), result, distSqr, idx))
            {
              return py::none();
            }
            return py::make_tuple(idx, mrpt::math::TPoint2D(result), distSqr);
          },
          "query"_a,
          "Closest point to a 2D query point, ignoring z (KD-tree search). Returns (index, point, "
          "squared distance), or None if none was found.")
      .def(
          "nn_multiple_search",
          [](const mrpt::maps::CPointsMap& mp, const mrpt::math::TPoint3D& query, size_t N)
          {
            std::vector<mrpt::math::TPoint3Df> pts;
            std::vector<float> distsSqr;
            std::vector<uint64_t> indices;
            mp.nn_multiple_search(mrpt::math::TPoint3Df(query), N, pts, distsSqr, indices);
            return nnResultsAsNumpy<3>(pts, distsSqr, indices);
          },
          "query"_a, "N"_a,
          "The N closest points to a 3D query point (KD-tree search), sorted by distance. Returns "
          "(indices: uint64 array, points: Nx3 float32 array, squared distances: float32 array).")
      .def(
          "nn_multiple_search",
          [](const mrpt::maps::CPointsMap& mp, const mrpt::math::TPoint2D& query, size_t N)
          {
            std::vector<mrpt::math::TPoint2Df> pts;
            std::vector<float> distsSqr;
            std::vector<uint64_t> indices;
            mp.nn_multiple_search(mrpt::math::TPoint2Df(query), N, pts, distsSqr, indices);
            return nnResultsAsNumpy<2>(pts, distsSqr, indices);
          },
          "query"_a, "N"_a,
          "The N closest points to a 2D query point, ignoring z (KD-tree search), sorted by "
          "distance. Returns (indices: uint64 array, points: Nx2 float32 array, squared "
          "distances: float32 array).")
      .def(
          "nn_radius_search",
          [](const mrpt::maps::CPointsMap& mp, const mrpt::math::TPoint3D& query,
             float search_radius_sqr, size_t maxPoints)
          {
            std::vector<mrpt::math::TPoint3Df> pts;
            std::vector<float> distsSqr;
            std::vector<uint64_t> indices;
            mp.nn_radius_search(
                mrpt::math::TPoint3Df(query), search_radius_sqr, pts, distsSqr, indices, maxPoints);
            return nnResultsAsNumpy<3>(pts, distsSqr, indices);
          },
          "query"_a, "search_radius_sqr"_a, "maxPoints"_a = 0,
          "The points within a distance of a 3D query point (KD-tree search). The radius is given "
          "squared; maxPoints=0 means no limit. Returns (indices: uint64 array, points: Nx3 "
          "float32 array, squared distances: float32 array).")
      .def(
          "nn_radius_search",
          [](const mrpt::maps::CPointsMap& mp, const mrpt::math::TPoint2D& query,
             float search_radius_sqr, size_t maxPoints)
          {
            std::vector<mrpt::math::TPoint2Df> pts;
            std::vector<float> distsSqr;
            std::vector<uint64_t> indices;
            mp.nn_radius_search(
                mrpt::math::TPoint2Df(query), search_radius_sqr, pts, distsSqr, indices, maxPoints);
            return nnResultsAsNumpy<2>(pts, distsSqr, indices);
          },
          "query"_a, "search_radius_sqr"_a, "maxPoints"_a = 0,
          "The points within a distance of a 2D query point, ignoring z (KD-tree search). The "
          "radius is given squared; maxPoints=0 means no limit. Returns (indices: uint64 array, "
          "points: Nx2 float32 array, squared distances: float32 array).")
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
      .def_static(
          "FromNumpyFields",
          [](const py::dict& fields)
          {
            using mrpt::maps::CGenericPointsMap;
            for (const char* c : {"x", "y", "z"})
            {
              if (!fields.contains(c))
              {
                throw py::key_error(std::string("missing field '") + c + "'");
              }
            }
            using FloatArray = py::array_t<float, py::array::c_style | py::array::forcecast>;
            const auto xs = py::cast<FloatArray>(fields["x"]);
            const auto ys = py::cast<FloatArray>(fields["y"]);
            const auto zs = py::cast<FloatArray>(fields["z"]);
            const auto n = xs.size();
            if (xs.ndim() != 1 || ys.ndim() != 1 || zs.ndim() != 1 || ys.size() != n ||
                zs.size() != n)
            {
              throw py::value_error("x, y and z must be 1D arrays of the same length");
            }
            auto mp = std::make_shared<CGenericPointsMap>();
            mp->resize(static_cast<size_t>(n));
            for (py::ssize_t i = 0; i < n; i++)
            {
              mp->setPointFast(static_cast<size_t>(i), xs.at(i), ys.at(i), zs.at(i));
            }
            for (const auto& [key, value] : fields)
            {
              const auto name = py::cast<std::string>(key);
              if (name == "x" || name == "y" || name == "z")
              {
                continue;
              }
              const auto arr = py::array::ensure(value);
              if (!arr || arr.ndim() != 1 || arr.size() != n)
              {
                throw py::value_error("field '" + name + "' must be a 1D array of length N");
              }
              const auto dt = arr.dtype();
              if (dt.is(py::dtype::of<float>()))
              {
                registerAndFillField<float>(
                    *mp, name, arr, &CGenericPointsMap::registerField_float,
                    &CGenericPointsMap::getPointsBufferRef_float_field);
              }
              else if (dt.is(py::dtype::of<double>()))
              {
                registerAndFillField<double>(
                    *mp, name, arr, &CGenericPointsMap::registerField_double,
                    &CGenericPointsMap::getPointsBufferRef_double_field);
              }
              else if (dt.is(py::dtype::of<uint16_t>()))
              {
                registerAndFillField<uint16_t>(
                    *mp, name, arr, &CGenericPointsMap::registerField_uint16,
                    &CGenericPointsMap::getPointsBufferRef_uint16_field);
              }
              else if (dt.is(py::dtype::of<uint8_t>()))
              {
                registerAndFillField<uint8_t>(
                    *mp, name, arr, &CGenericPointsMap::registerField_uint8,
                    &CGenericPointsMap::getPointsBufferRef_uint8_field);
              }
              else if (dt.is(py::dtype::of<uint32_t>()))
              {
                registerAndFillField<uint32_t>(
                    *mp, name, arr, &CGenericPointsMap::registerField_uint32,
                    &CGenericPointsMap::getPointsBufferRef_uint32_field);
              }
              else
              {
                throw py::type_error(
                    "field '" + name +
                    "' must have dtype float32, float64, uint16, uint8 or uint32");
              }
            }
            mp->mark_as_modified();
            return mp;
          },
          "fields"_a,
          "Builds a map from a dict of 1D arrays of the same length, as returned by "
          "getPointFieldsAsNumpy(): x, y and z (converted to float32) and other fields, "
          "registered with the type given by their dtype (float32, float64, uint16, uint8 or "
          "uint32).")
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
          "setFromNumpy",
          [](mrpt::maps::COccupancyGridMap2D& g,
             const py::array_t<float, py::array::c_style | py::array::forcecast>& arr)
          {
            if (arr.ndim() != 2 || arr.shape(0) != static_cast<py::ssize_t>(g.getSizeY()) ||
                arr.shape(1) != static_cast<py::ssize_t>(g.getSizeX()))
            {
              throw py::value_error("The array must have shape (getSizeY(), getSizeX())");
            }
            const auto buf = arr.unchecked<2>();
            for (py::ssize_t row = 0; row < buf.shape(0); row++)
            {
              for (py::ssize_t col = 0; col < buf.shape(1); col++)
              {
                g.setCell(static_cast<int>(col), static_cast<int>(row), buf(row, col));
              }
            }
          },
          "array"_a,
          "Sets all the cells from an HxW array, with the same layout and values as getAsNumpy() "
          "(0=occupied, 1=free).")
      .def(
          "laserScanSimulator", &mrpt::maps::COccupancyGridMap2D::laserScanSimulator,
          "inout_Scan"_a, "robotPose"_a, "threshold"_a = 0.6f, "N"_a = 361, "noiseStd"_a = 0.0f,
          "decimation"_a = 1, "angleNoiseStd"_a = 0.0f,
          "Simulates a 2D laser scan from robotPose (CPose2D) into this grid. inout_Scan "
          "(CObservation2DRangeScan) must have its aperture, maxRange, sensorPose and rightToLeft "
          "set, and gets the N simulated ranges. threshold is the minimum occupancy of a cell to "
          "be an obstacle; noiseStd and angleNoiseStd are the sigmas of the range and angle "
          "noise; only one ray out of 'decimation' is simulated.")
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

  // -------------------------------------------------------------------------
  // Occupied voxels of 3D occupancy maps, as NumPy arrays
  // -------------------------------------------------------------------------
  registeredClass<Grid3D>().def(
      "getOccupiedVoxelsAsNumpy",
      [](const Grid3D& g, float occupiedThreshold)
      {
        const auto& grid = g.m_grid;
        const double resXY = grid.getResolutionXY();
        const double resZ = grid.getResolutionZ();
        std::vector<mrpt::math::TPoint3Df> centers;
        for (size_t cz = 0; cz < grid.getSizeZ(); cz++)
        {
          for (size_t cy = 0; cy < grid.getSizeY(); cy++)
          {
            for (size_t cx = 0; cx < grid.getSizeX(); cx++)
            {
              const float occ =
                  1.0f - g.getCellFreeness(
                             static_cast<unsigned int>(cx), static_cast<unsigned int>(cy),
                             static_cast<unsigned int>(cz));
              if (occ > occupiedThreshold)
              {
                centers.emplace_back(
                    static_cast<float>(grid.idx2x(static_cast<int>(cx)) + 0.5 * resXY),
                    static_cast<float>(grid.idx2y(static_cast<int>(cy)) + 0.5 * resXY),
                    static_cast<float>(grid.idx2z(static_cast<int>(cz)) + 0.5 * resZ));
              }
            }
          }
        }
        const auto n = static_cast<py::ssize_t>(centers.size());
        py::array_t<float> arr(std::vector<py::ssize_t>{n, 3});
        auto buf = arr.mutable_unchecked<2>();
        for (py::ssize_t i = 0; i < n; i++)
        {
          buf(i, 0) = centers[i].x;
          buf(i, 1) = centers[i].y;
          buf(i, 2) = centers[i].z;
        }
        return arr;
      },
      "occupiedThreshold"_a = 0.5f,
      "Returns the centers of the voxels with an occupancy probability above the threshold, as "
      "an Nx3 float32 array.");

  registeredClass<mrpt::maps::COctoMap>().def(
      "getOccupiedVoxelsAsNumpy",
      [](mrpt::maps::COctoMap& om)
      {
        // Only build the occupied voxels, restoring the rendering options afterwards:
        const auto oldOptions = om.renderingOptions;
        om.renderingOptions.generateGridLines = false;
        om.renderingOptions.generateOccupiedVoxels = true;
        om.renderingOptions.generateFreeVoxels = false;
        mrpt::viz::COctoMapVoxels voxels;
        om.getAsOctoMapVoxels(voxels);
        om.renderingOptions = oldOptions;

        const size_t setIdx = mrpt::viz::VOXEL_SET_OCCUPIED;
        const auto n = static_cast<py::ssize_t>(
            voxels.getVoxelSetCount() > setIdx ? voxels.getVoxelCount(setIdx) : 0);
        py::array_t<float> centers(std::vector<py::ssize_t>{n, 3});
        py::array_t<float> sizes(std::vector<py::ssize_t>{n});
        auto bufCenters = centers.mutable_unchecked<2>();
        auto bufSizes = sizes.mutable_unchecked<1>();
        for (py::ssize_t i = 0; i < n; i++)
        {
          const auto& v = voxels.getVoxel(setIdx, static_cast<size_t>(i));
          bufCenters(i, 0) = v.coords.x;
          bufCenters(i, 1) = v.coords.y;
          bufCenters(i, 2) = v.coords.z;
          bufSizes(i) = v.side_length;
        }
        return py::make_tuple(centers, sizes);
      },
      "Returns the occupied leaf voxels (occupancy >= 0.5) as a tuple (centers: Nx3 float32 "
      "array, side lengths: float32 array). Pruned leaves may be larger than the resolution.");

  // -------------------------------------------------------------------------
  // Insertion and likelihood options of the maps
  // -------------------------------------------------------------------------
  using mrpt::config::CLoadableOptions;
  {
    using G = mrpt::maps::COccupancyGridMap2D;
    const auto scope = py::type::of<G>();
    using Ins = G::TInsertionOptions;
    py::class_<Ins, CLoadableOptions, std::shared_ptr<Ins>>(
        scope, "TInsertionOptions",
        "Options for inserting observations into a COccupancyGridMap2D.")
        .def(py::init<>(), "Default constructor.")
        .def_readwrite(
            "mapAltitude", &Ins::mapAltitude,
            "Altitude (z) of the 2D scans to insert (0.01 m tolerance), if useMapAltitude is true.")
        .def_readwrite(
            "useMapAltitude", &Ins::useMapAltitude,
            "Only insert 2D scans at mapAltitude (default: false).")
        .def_readwrite(
            "maxDistanceInsertion", &Ins::maxDistanceInsertion,
            "Largest distance at which cells are updated (default: 15 m).")
        .def_readwrite(
            "maxOccupancyUpdateCertainty", &Ins::maxOccupancyUpdateCertainty,
            "Value in [0.5, 1] for the Bayesian update of occupied cells (default: 0.65).")
        .def_readwrite(
            "maxFreenessUpdateCertainty", &Ins::maxFreenessUpdateCertainty,
            "Value in [0.5, 1] for the update of free cells (default: 0, the same as "
            "maxOccupancyUpdateCertainty).")
        .def_readwrite(
            "maxFreenessInvalidRanges", &Ins::maxFreenessInvalidRanges,
            "Like maxFreenessUpdateCertainty, for invalid ranges (default: 0, the same as "
            "maxOccupancyUpdateCertainty).")
        .def_readwrite(
            "considerInvalidRangesAsFreeSpace", &Ins::considerInvalidRangesAsFreeSpace,
            "Use invalid ranges (no echo) as free space, if the previous and next rays are also "
            "invalid (default: true).")
        .def_readwrite(
            "decimation", &Ins::decimation, "Decimation of range scans (default: 1, all rays).")
        .def_readwrite(
            "horizontalTolerance", &Ins::horizontalTolerance,
            "Tolerance in pitch and roll for a scan to be horizontal, in radians.")
        .def_readwrite(
            "CFD_features_gaussian_size", &Ins::CFD_features_gaussian_size,
            "Sigma of the Gaussian filter of getAsImageFiltered() (default: 1, 0: disabled).")
        .def_readwrite(
            "CFD_features_median_size", &Ins::CFD_features_median_size,
            "Size of the median filter of getAsImageFiltered() (default: 3, 0: disabled).")
        .def_readwrite(
            "wideningBeamsWithDistance", &Ins::wideningBeamsWithDistance,
            "Rays widen with distance, like real laser beams (default: false).")
        .def_readwrite(
            "raytraceStepSizeInCellUnits", &Ins::raytraceStepSizeInCellUnits,
            "Unused: kept for backwards compatibility.");

    py::enum_<G::TLikelihoodMethod>(
        scope, "TLikelihoodMethod", "Method to compute the likelihood of observations.")
        .value("lmMeanInformation", G::lmMeanInformation)
        .value("lmRayTracing", G::lmRayTracing)
        .value("lmConsensus", G::lmConsensus)
        .value("lmCellsDifference", G::lmCellsDifference)
        .value("lmLikelihoodField_Thrun", G::lmLikelihoodField_Thrun)
        .value("lmLikelihoodField_II", G::lmLikelihoodField_II)
        .value("lmConsensusOWA", G::lmConsensusOWA)
        .export_values();

    using Lik = G::TLikelihoodOptions;
    py::class_<Lik, CLoadableOptions, std::shared_ptr<Lik>>(
        scope, "TLikelihoodOptions",
        "Options for computing the likelihood of observations in a COccupancyGridMap2D.")
        .def(py::init<>(), "Default constructor.")
        .def_readwrite(
            "likelihoodMethod", &Lik::likelihoodMethod,
            "Likelihood method (default: lmLikelihoodField_Thrun).")
        .def_readwrite(
            "LF_stdHit", &Lik::LF_stdHit, "[LikelihoodField] Range sigma (default: 0.35).")
        .def_readwrite(
            "LF_zHit", &Lik::LF_zHit, "[LikelihoodField] Weight of hits (default: 0.95).")
        .def_readwrite(
            "LF_zRandom", &Lik::LF_zRandom,
            "[LikelihoodField] Weight of random measurements (default: 0.05).")
        .def_readwrite(
            "LF_maxRange", &Lik::LF_maxRange,
            "[LikelihoodField] Maximum range of the sensor (default: 81 m).")
        .def_readwrite(
            "LF_decimation", &Lik::LF_decimation,
            "[LikelihoodField] Decimation of the points of a scan (default: 5, 1: none).")
        .def_readwrite(
            "LF_maxCorrsDistance", &Lik::LF_maxCorrsDistance,
            "[LikelihoodField] Maximum distance to search correspondences of each point.")
        .def_readwrite(
            "LF_useSquareDist", &Lik::LF_useSquareDist,
            "[LikelihoodField] Use exp(-0.5*d^4/s^2) instead of exp(-0.5*d^2/s^2) (default: "
            "false).")
        .def_readwrite(
            "LF_alternateAverageMethod", &Lik::LF_alternateAverageMethod,
            "[LikelihoodField] Average the likelihood of the ranges instead of multiplying them "
            "(default: false).")
        .def_readwrite("MI_exponent", &Lik::MI_exponent, "[MI] Exponent (default: 2.5).")
        .def_readwrite(
            "MI_skip_rays", &Lik::MI_skip_rays, "[MI] Use one out of N rays (default: 10).")
        .def_readwrite(
            "MI_ratio_max_distance", &Lik::MI_ratio_max_distance,
            "[MI] Ratio of the maximum distance, with respect to the insertion one (default: 1.5).")
        .def_readwrite(
            "rayTracing_useDistanceFilter", &Lik::rayTracing_useDistanceFilter,
            "[rayTracing] Unused: kept for backwards compatibility.")
        .def_readwrite(
            "rayTracing_decimation", &Lik::rayTracing_decimation,
            "[rayTracing] Simulate one out of N rays (default: 10).")
        .def_readwrite(
            "rayTracing_stdHit", &Lik::rayTracing_stdHit, "[rayTracing] Range sigma (default: 1).")
        .def_readwrite(
            "consensus_takeEachRange", &Lik::consensus_takeEachRange,
            "[Consensus] Use one out of N ranges (default: 1).")
        .def_readwrite(
            "consensus_pow", &Lik::consensus_pow,
            "[Consensus] Power of the likelihood (default: 5).")
        .def_readwrite(
            "OWA_weights", &Lik::OWA_weights,
            "[OWA] Weights of the sorted likelihood values (the largest first). Assign a whole "
            "list to change it.")
        .def_readwrite(
            "enableLikelihoodCache", &Lik::enableLikelihoodCache,
            "Use a cache of likelihood values, for the likelihood field methods (default: true).");

    registeredClass<G>()
        .def_readwrite(
            "insertionOptions", &G::insertionOptions,
            "Options for inserting observations (TInsertionOptions).")
        .def_readwrite(
            "likelihoodOptions", &G::likelihoodOptions,
            "Options for computing the likelihood of observations (TLikelihoodOptions).");
  }
  {
    using Ins = Grid3D::TInsertionOptions;
    const auto scope = py::type::of<Grid3D>();
    py::class_<Ins, CLoadableOptions, std::shared_ptr<Ins>>(
        scope, "TInsertionOptions",
        "Options for inserting observations into a COccupancyGridMap3D.")
        .def(py::init<>(), "Default constructor.")
        .def_readwrite(
            "maxDistanceInsertion", &Ins::maxDistanceInsertion,
            "Largest distance at which voxels are updated (default: 15 m).")
        .def_readwrite(
            "maxOccupancyUpdateCertainty", &Ins::maxOccupancyUpdateCertainty,
            "Value in [0.5, 1] for the Bayesian update of occupied voxels (default: 0.65).")
        .def_readwrite(
            "maxFreenessUpdateCertainty", &Ins::maxFreenessUpdateCertainty,
            "Value in [0.5, 1] for the update of free voxels (default: 0, the same as "
            "maxOccupancyUpdateCertainty).")
        .def_readwrite(
            "decimation_3d_range", &Ins::decimation_3d_range,
            "Decimation of 3D range scans: keeps the minimum range of each NxN block (default: 8).")
        .def_readwrite(
            "decimation", &Ins::decimation,
            "Decimation of point clouds and 2D range scans (default: 1).")
        .def_readwrite(
            "raytraceEmptyCells", &Ins::raytraceEmptyCells,
            "Mark the voxels along the rays as free (default: true), or only update the end "
            "points.");

    py::enum_<Grid3D::TLikelihoodMethod>(
        scope, "TLikelihoodMethod", "Method to compute the likelihood of observations.")
        .value("lmLikelihoodField_Thrun", Grid3D::lmLikelihoodField_Thrun)
        .value("lmRayTracing", Grid3D::lmRayTracing)
        .export_values();

    using Lik = Grid3D::TLikelihoodOptions;
    py::class_<Lik, CLoadableOptions, std::shared_ptr<Lik>>(
        scope, "TLikelihoodOptions",
        "Options for computing the likelihood of observations in a COccupancyGridMap3D (not "
        "implemented yet for this map: likelihoods are always 0).")
        .def(py::init<>(), "Default constructor.")
        .def_readwrite(
            "likelihoodMethod", &Lik::likelihoodMethod,
            "Likelihood method (default: lmLikelihoodField_Thrun).")
        .def_readwrite(
            "LF_stdHit", &Lik::LF_stdHit, "[LikelihoodField] Range sigma (default: 0.35).")
        .def_readwrite(
            "LF_zHit", &Lik::LF_zHit, "[LikelihoodField] Weight of hits (default: 0.95).")
        .def_readwrite(
            "LF_zRandom", &Lik::LF_zRandom,
            "[LikelihoodField] Weight of random measurements (default: 0.05).")
        .def_readwrite(
            "LF_maxRange", &Lik::LF_maxRange,
            "[LikelihoodField] Maximum range of the sensor (default: 20 m).")
        .def_readwrite(
            "LF_decimation", &Lik::LF_decimation,
            "[LikelihoodField] Decimation of the points (default: 1, none).")
        .def_readwrite(
            "LF_maxCorrsDistance", &Lik::LF_maxCorrsDistance,
            "[LikelihoodField] Maximum distance to search correspondences of each point.")
        .def_readwrite(
            "LF_useSquareDist", &Lik::LF_useSquareDist,
            "[LikelihoodField] Use exp(-0.5*d^4/s^2) instead of exp(-0.5*d^2/s^2) (default: "
            "false).")
        .def_readwrite(
            "rayTracing_decimation", &Lik::rayTracing_decimation,
            "[rayTracing] Simulate one out of N rays (default: 10).")
        .def_readwrite(
            "rayTracing_stdHit", &Lik::rayTracing_stdHit, "[rayTracing] Range sigma (default: 1).");

    registeredClass<Grid3D>()
        .def_readwrite(
            "insertionOptions", &Grid3D::insertionOptions,
            "Options for inserting observations (TInsertionOptions).")
        .def_readwrite(
            "likelihoodOptions", &Grid3D::likelihoodOptions,
            "Options for computing the likelihood of observations (TLikelihoodOptions).");
  }
  {
    using Ins = HMap::TInsertionOptions;
    py::class_<Ins, CLoadableOptions, std::shared_ptr<Ins>>(
        py::type::of<HMap>(), "TInsertionOptions",
        "Options for inserting observations into a CHeightGridMap2D.")
        .def(py::init<>(), "Default constructor.")
        .def_readwrite(
            "filterByHeight", &Ins::filterByHeight,
            "Only insert points with z in [z_min, z_max], relative to the robot (default: false).")
        .def_readwrite(
            "z_min", &Ins::z_min, "Minimum z, if filterByHeight is true (default: -0.5).")
        .def_readwrite("z_max", &Ins::z_max, "Maximum z, if filterByHeight is true (default: 0.5).")
        .def_readwrite("colorMap", &Ins::colorMap, "Color map for the 3D view (default: cmJET).");

    registeredClass<HMap>().def_readwrite(
        "insertionOptions", &HMap::insertionOptions,
        "Options for inserting observations (TInsertionOptions).");
  }
  {
    using OM = mrpt::maps::COctoMap;
    using Ins = OM::TInsertionOptions;
    const auto scope = py::type::of<OM>();
    // No constructor: the sensor model parameters are only stored in a map.
    py::class_<Ins, CLoadableOptions, std::shared_ptr<Ins>>(
        scope, "TInsertionOptions",
        "Options for inserting observations into a COctoMap. Access them through "
        "COctoMap.insertionOptions: the sensor model parameters are stored in the map itself.")
        .def_readwrite(
            "maxrange", &Ins::maxrange,
            "Maximum range of the inserted rays (default: -1, the complete rays).")
        .def_readwrite(
            "pruning", &Ins::pruning, "Prune the tree after each insertion (default: true).")
        .def_property(
            "occupancyThres", &Ins::getOccupancyThres, &Ins::setOccupancyThres,
            "Occupancy probability threshold of the sensor model (default: 0.5).")
        .def_property(
            "probHit", &Ins::getProbHit, &Ins::setProbHit,
            "Probability of a hit in the sensor model (default: 0.7).")
        .def_property(
            "probMiss", &Ins::getProbMiss, &Ins::setProbMiss,
            "Probability of a miss in the sensor model (default: 0.4).")
        .def_property(
            "clampingThresMin", &Ins::getClampingThresMin, &Ins::setClampingThresMin,
            "Minimum occupancy probability for clamping (default: 0.1192).")
        .def_property(
            "clampingThresMax", &Ins::getClampingThresMax, &Ins::setClampingThresMax,
            "Maximum occupancy probability for clamping (default: 0.971).");

    using Lik = OM::TLikelihoodOptions;
    py::class_<Lik, CLoadableOptions, std::shared_ptr<Lik>>(
        scope, "TLikelihoodOptions",
        "Options for computing the likelihood of observations in a COctoMap.")
        .def(py::init<>(), "Default constructor.")
        .def_readwrite(
            "decimation", &Lik::decimation, "Use one out of N rays (default: 1, all of them).");

    registeredClass<OM>()
        .def_property_readonly(
            "insertionOptions", [](OM& om) -> Ins& { return om.insertionOptions; },
            py::return_value_policy::reference_internal,
            "Options for inserting observations (TInsertionOptions): modify its fields in place.")
        .def_readwrite(
            "likelihoodOptions", &OM::likelihoodOptions,
            "Options for computing the likelihood of observations (TLikelihoodOptions).");
  }
  {
    using Ins = mrpt::maps::TVoxelMap_InsertionOptions;
    py::class_<Ins, CLoadableOptions, std::shared_ptr<Ins>>(
        m, "TVoxelMap_InsertionOptions",
        "Options for inserting observations into a CVoxelMap or CVoxelMapRGB.")
        .def(py::init<>(), "Default constructor.")
        .def_readwrite(
            "max_range", &Ins::max_range, "Maximum range of the inserted rays (<0: no limit).")
        .def_readwrite(
            "prob_miss", &Ins::prob_miss,
            "Occupancy probability of a voxel crossed by a ray, < 0.5 (default: 0.45).")
        .def_readwrite(
            "prob_hit", &Ins::prob_hit,
            "Occupancy probability of a voxel where a ray ends, > 0.5 (default: 0.65).")
        .def_readwrite(
            "clamp_min", &Ins::clamp_min, "Minimum occupancy probability (default: 0.10).")
        .def_readwrite(
            "clamp_max", &Ins::clamp_max, "Maximum occupancy probability (default: 0.95).")
        .def_readwrite(
            "ray_trace_free_space", &Ins::ray_trace_free_space,
            "Mark the voxels along the rays as free (default: true).")
        .def_readwrite("decimation", &Ins::decimation, "Use one out of N points (default: 1).")
        .def_readwrite(
            "remove_voxels_farther_than", &Ins::remove_voxels_farther_than,
            "If not 0, remove the voxels farther than this L1 distance, in meters (default: 0).");

    using Lik = mrpt::maps::TVoxelMap_LikelihoodOptions;
    py::class_<Lik, CLoadableOptions, std::shared_ptr<Lik>>(
        m, "TVoxelMap_LikelihoodOptions",
        "Options for computing the likelihood of observations in a CVoxelMap or CVoxelMapRGB.")
        .def(py::init<>(), "Default constructor.")
        .def_readwrite(
            "decimate_up_to", &Lik::decimate_up_to,
            "Use at most this number of points (default: 0, values <= 1 mean all of them).")
        .def_readwrite(
            "occupiedThreshold", &Lik::occupiedThreshold,
            "Minimum occupancy for a voxel to be occupied (default: 0.60).");

    const auto addVoxelMapOptions = [](auto clsTag)
    {
      using T = typename decltype(clsTag)::type;
      registeredClass<T>()
          .def_readwrite(
              "insertionOptions", &T::insertionOptions,
              "Options for inserting observations (TVoxelMap_InsertionOptions).")
          .def_readwrite(
              "likelihoodOptions", &T::likelihoodOptions,
              "Options for computing the likelihood of observations "
              "(TVoxelMap_LikelihoodOptions).");
    };
    addVoxelMapOptions(type_tag<mrpt::maps::CVoxelMap>{});
    addVoxelMapOptions(type_tag<mrpt::maps::CVoxelMapRGB>{});
  }
}
