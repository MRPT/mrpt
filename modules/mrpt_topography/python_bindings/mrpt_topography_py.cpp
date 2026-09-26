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
#include <mrpt/math/TPose3D.h>
#include <mrpt/topography/conversions.h>
#include <mrpt/topography/data_types.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cctype>
#include <optional>

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
  // -------------------------------------------------------------------------
  // TEllipsoid - reference ellipsoid
  // -------------------------------------------------------------------------
  py::class_<mrpt::topography::TEllipsoid> ellipsoid(m, "TEllipsoid");
  ellipsoid.def(py::init<>())
      .def(py::init<double, double, std::string>(), "sa"_a, "sb"_a, "name"_a)
      .def_readwrite("sa", &mrpt::topography::TEllipsoid::sa, "Largest semiaxis (meters)")
      .def_readwrite("sb", &mrpt::topography::TEllipsoid::sb, "Smallest semiaxis (meters)")
      .def_readwrite("name", &mrpt::topography::TEllipsoid::name)
      .def(
          "__repr__",
          [](const mrpt::topography::TEllipsoid& e)
          {
            return "TEllipsoid(name='" + e.name + "', sa=" + std::to_string(e.sa) +
                   ", sb=" + std::to_string(e.sb) + ")";
          });
  {
    using E = mrpt::topography::TEllipsoid;
    const std::pair<const char*, E (*)()> factories[] = {
        {                   "Ellipsoid_WGS84",                    &E::Ellipsoid_WGS84},
        {                   "Ellipsoid_WGS72",                    &E::Ellipsoid_WGS72},
        {                   "Ellipsoid_WGS66",                    &E::Ellipsoid_WGS66},
        {            "Ellipsoid_Walbeck_1817",             &E::Ellipsoid_Walbeck_1817},
        {       "Ellipsoid_Sudamericano_1969",        &E::Ellipsoid_Sudamericano_1969},
        {"Ellipsoid_Nuevo_Internacional_1967", &E::Ellipsoid_Nuevo_Internacional_1967},
        { "Ellipsoid_Mercury_Modificado_1968",  &E::Ellipsoid_Mercury_Modificado_1968},
        {            "Ellipsoid_Mercury_1960",             &E::Ellipsoid_Mercury_1960},
        {          "Ellipsoid_Krasovsky_1940",           &E::Ellipsoid_Krasovsky_1940},
        {      "Ellipsoid_Internacional_1924",       &E::Ellipsoid_Internacional_1924},
        {      "Ellipsoid_Internacional_1909",       &E::Ellipsoid_Internacional_1909},
        {              "Ellipsoid_Hough_1960",               &E::Ellipsoid_Hough_1960},
        {            "Ellipsoid_Helmert_1906",             &E::Ellipsoid_Helmert_1906},
        {            "Ellipsoid_Hayford_1909",             &E::Ellipsoid_Hayford_1909},
        {                   "Ellipsoid_GRS80",                    &E::Ellipsoid_GRS80},
        {            "Ellipsoid_Fischer_1968",             &E::Ellipsoid_Fischer_1968},
        {            "Ellipsoid_Fischer_1960",             &E::Ellipsoid_Fischer_1960},
        {             "Ellipsoid_Clarke_1880",              &E::Ellipsoid_Clarke_1880},
        {             "Ellipsoid_Clarke_1866",              &E::Ellipsoid_Clarke_1866},
        {             "Ellipsoid_Bessel_1841",              &E::Ellipsoid_Bessel_1841},
        {    "Ellipsoid_Airy_Modificado_1965",     &E::Ellipsoid_Airy_Modificado_1965},
        {               "Ellipsoid_Airy_1830",                &E::Ellipsoid_Airy_1830},
    };
    for (const auto& [name, fn] : factories)
    {
      ellipsoid.def_static(name, fn);
    }
  }

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
      [](const mrpt::math::TPoint3D& gc, const mrpt::topography::TEllipsoid& ellip)
      {
        mrpt::topography::TGeodeticCoords gd;
        mrpt::topography::geocentricToGeodetic(gc, gd, ellip);
        return gd;
      },
      "geocentric"_a, "ellipsoid"_a = mrpt::topography::TEllipsoid::Ellipsoid_WGS84(),
      "Convert geocentric ECEF TPoint3D (x,y,z) to geodetic coordinates (default: WGS84)");

  m.def(
      "geodeticToGeocentric",
      [](const mrpt::topography::TGeodeticCoords& gd, const mrpt::topography::TEllipsoid& ellip)
      {
        mrpt::topography::TGeocentricCoords gc;
        mrpt::topography::geodeticToGeocentric(gd, gc, ellip);
        return gc;
      },
      "geodetic"_a, "ellipsoid"_a,
      "Convert geodetic (lat,lon,h) to geocentric (ECEF) TPoint3D for the given ellipsoid");

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
      "ENUToGeodetic_WGS84",
      [](const mrpt::math::TPoint3D& enu, const mrpt::topography::TGeodeticCoords& origin)
      {
        mrpt::topography::TGeodeticCoords gd;
        mrpt::topography::ENUToGeodetic_WGS84(enu, gd, origin);
        return gd;
      },
      "enu"_a, "origin"_a,
      "Convert local ENU coordinates (meters) relative to origin to a WGS84 geodetic point. "
      "Exact inverse of geodeticToENU_WGS84()");

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

  m.def(
      "ENU_axes_from_WGS84",
      [](const mrpt::topography::TGeodeticCoords& coords, bool onlyAngles)
      {
        mrpt::math::TPose3D enu;
        mrpt::topography::ENU_axes_from_WGS84(coords, enu, onlyAngles);
        return enu;
      },
      "coords"_a, "only_angles"_a = false,
      "Returns the East-North-Up frame at the given point, as a TPose3D in ECEF coordinates");

  // -------------------------------------------------------------------------
  // UTM conversions
  // -------------------------------------------------------------------------
  m.def(
      "geodeticToUTM",
      [](const mrpt::topography::TGeodeticCoords& gd, const mrpt::topography::TEllipsoid& ellip)
      {
        mrpt::topography::TUTMCoords utm;
        int zone = 0;
        char band = 0;
        mrpt::topography::geodeticToUTM(gd, utm, zone, band, ellip);
        return py::make_tuple(utm, zone, std::string(1, band));
      },
      "geodetic"_a, "ellipsoid"_a = mrpt::topography::TEllipsoid::Ellipsoid_WGS84(),
      "Convert geodetic coordinates to UTM. Returns (utm: TPoint3D, zone: int, band: str); "
      "utm.z is the height");

  m.def(
      "UTMToGeodetic",
      [](const mrpt::math::TPoint3D& utm, int zone, const std::optional<std::string>& hemisphere,
         const mrpt::topography::TEllipsoid& ellip, const std::optional<std::string>& band)
      {
        if (hemisphere.has_value() == band.has_value())
        {
          throw std::invalid_argument("Pass exactly one of hemisphere ('N'/'S') or band");
        }
        char hem = 0;
        if (hemisphere)
        {
          if (hemisphere->size() != 1)
          {
            throw std::invalid_argument("hemisphere must be 'N' or 'S'");
          }
          hem = (*hemisphere)[0];
        }
        else
        {
          // UTM latitude bands: C-M are south of the equator, N-X north.
          const std::string south = "CDEFGHJKLM";
          const std::string north = "NPQRSTUVWX";
          const char b = band->size() == 1 ? static_cast<char>(std::toupper((*band)[0])) : 0;
          if (b != 0 && south.find(b) != std::string::npos)
          {
            hem = 'S';
          }
          else if (b != 0 && north.find(b) != std::string::npos)
          {
            hem = 'N';
          }
          else
          {
            throw std::invalid_argument("band must be a UTM latitude band letter (C-X)");
          }
        }
        mrpt::topography::TGeodeticCoords gd;
        mrpt::topography::UTMToGeodetic(utm, zone, hem, gd, ellip);
        return gd;
      },
      "utm"_a, "zone"_a, "hemisphere"_a = std::nullopt,
      "ellipsoid"_a = mrpt::topography::TEllipsoid::Ellipsoid_WGS84(), "band"_a = std::nullopt,
      "Convert UTM coordinates (utm.z is the height) to geodetic coordinates. Give either the "
      "hemisphere ('N' or 'S') or the latitude band returned by geodeticToUTM() (band=...).");
}
