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

#include <mrpt/math/TPoint3D.h>
#include <mrpt/topography/conversions.h>
#include <mrpt/topography/data_types.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_MODULE(_bindings, m)
{
  m.doc() = "Python bindings for mrpt::topography — geodetic coordinate conversions";

  // -------------------------------------------------------------------------
  // TCoords — degrees/minutes/seconds coordinate type
  // -------------------------------------------------------------------------
  py::class_<mrpt::topography::TCoords>(m, "TCoords")
      .def(py::init<>())
      .def(py::init<double>(), "decimal_deg"_a)
      .def(py::init<int, int, double>(), "deg"_a, "min"_a, "sec"_a)
      .def_readwrite("decimal_value", &mrpt::topography::TCoords::decimal_value)
      .def("getDecimalValue", &mrpt::topography::TCoords::getDecimalValue)
      .def("setFromDecimal", &mrpt::topography::TCoords::setFromDecimal)
      .def(
          "getDegMinSec",
          [](const mrpt::topography::TCoords& c)
          {
            int deg, min;
            double sec;
            c.getDegMinSec(deg, min, sec);
            return py::make_tuple(deg, min, sec);
          },
          "Returns (degrees, minutes, seconds) tuple")
      .def(
          "__repr__",
          [](const mrpt::topography::TCoords& c)
          {
            int d, m;
            double s;
            c.getDegMinSec(d, m, s);
            return std::to_string(d) + "° " + std::to_string(m) + "' " + std::to_string(s) + "\"";
          });

  // -------------------------------------------------------------------------
  // TGeodeticCoords — (latitude, longitude, height) WGS84
  // -------------------------------------------------------------------------
  py::class_<mrpt::topography::TGeodeticCoords>(m, "TGeodeticCoords")
      .def(py::init<>())
      .def(py::init<double, double, double>(), "lat_deg"_a, "lon_deg"_a, "height_m"_a)
      .def_readwrite(
          "lat", &mrpt::topography::TGeodeticCoords::lat, "Latitude in degrees (TCoords)")
      .def_readwrite(
          "lon", &mrpt::topography::TGeodeticCoords::lon, "Longitude in degrees (TCoords)")
      .def_readwrite(
          "height", &mrpt::topography::TGeodeticCoords::height, "Geodetic height in meters")
      .def(
          "__repr__",
          [](const mrpt::topography::TGeodeticCoords& c)
          {
            return "TGeodeticCoords(lat=" + std::to_string(c.lat.getDecimalValue()) +
                   "°, lon=" + std::to_string(c.lon.getDecimalValue()) +
                   "°, h=" + std::to_string(c.height) + "m)";
          });

  // -------------------------------------------------------------------------
  // Geodetic ↔ Geocentric (ECEF) conversions
  // -------------------------------------------------------------------------
  m.def(
      "geodeticToGeocentric_WGS84",
      [](const mrpt::topography::TGeodeticCoords& gd)
      {
        mrpt::math::TPoint3D gc;
        mrpt::topography::geodeticToGeocentric_WGS84(gd, gc);
        return gc;
      },
      "geodetic"_a,
      "Convert WGS84 geodetic (lat,lon,h) to geocentric (ECEF) TPoint3D (x,y,z) in meters");

  m.def(
      "geocentricToGeodetic",
      [](const mrpt::math::TPoint3D& gc)
      {
        mrpt::topography::TGeodeticCoords gd;
        mrpt::topography::geocentricToGeodetic(gc, gd);
        return gd;
      },
      "geocentric"_a, "Convert geocentric ECEF TPoint3D (x,y,z) to WGS84 TGeodeticCoords");

  // -------------------------------------------------------------------------
  // Geodetic ↔ ENU (local East-North-Up) conversions
  // -------------------------------------------------------------------------
  m.def(
      "geodeticToENU_WGS84",
      [](const mrpt::topography::TGeodeticCoords& point,
         const mrpt::topography::TGeodeticCoords& origin)
      {
        mrpt::math::TPoint3D enu;
        mrpt::topography::geodeticToENU_WGS84(point, enu, origin);
        return enu;
      },
      "point"_a, "origin"_a,
      "Convert WGS84 geodetic point to local ENU coordinates (meters) relative to origin");

  m.def(
      "ENUToGeocentric",
      [](const mrpt::math::TPoint3D& enu, const mrpt::topography::TGeodeticCoords& origin)
      {
        mrpt::math::TPoint3D gc;
        mrpt::topography::ENUToGeocentric(
            enu, origin, gc, mrpt::topography::TEllipsoid::Ellipsoid_WGS84());
        return gc;
      },
      "enu"_a, "origin"_a,
      "Convert ENU local coordinates to ECEF geocentric, given a WGS84 reference origin");
}
