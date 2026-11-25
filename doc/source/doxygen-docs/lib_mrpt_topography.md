\defgroup mrpt_topography_grp [mrpt-topography]

Topographic functions, geoid transformations.

[TOC]

# Library mrpt-topography


This C++ library is part of MRPT and can be installed in Debian-based systems
with:

		sudo apt install libmrpt-topography-dev

Read also [how to import MRPT into your CMake scripts](mrpt_from_cmake.html).

Conversion and useful data structures to handle topographic data, perform
geoid transformations, geocentric coordinates, etc...


See mrpt::topography

## Overview

Use this module to bring GNSS (GPS) readings into the Cartesian frames used
by mapping and localization algorithms. It converts between:

- **Geodetic** coordinates: latitude, longitude (degrees) and ellipsoidal
  height (meters), on the WGS84 or any other reference ellipsoid.
- **Geocentric** (ECEF) coordinates: X, Y, Z with origin at the Earth center.
- **ENU** (East-North-Up) local coordinates, relative to a given geodetic
  origin: the usual choice for local maps.
- **UTM** projected coordinates, with their zone and latitude band.

It also implements datum transformations (7 and 10-parameter, Helmert 2D and
3D, etc.).

## Key types and functions

- mrpt::topography::TGeodeticCoords: latitude, longitude and height.
- mrpt::topography::TEllipsoid: reference ellipsoid parameters, with
  predefined ellipsoids like `TEllipsoid::Ellipsoid_WGS84()`.
- mrpt::topography::geodeticToENU_WGS84() and its inverse
  mrpt::topography::ENUToGeodetic_WGS84().
- mrpt::topography::geodeticToGeocentric_WGS84() and
  mrpt::topography::geocentricToENU_WGS84().
- mrpt::topography::geodeticToUTM() and mrpt::topography::UTMToGeodetic().

## Example

```cpp
#include <mrpt/topography/conversions.h>

// Latitude and longitude in degrees, height in meters:
const mrpt::topography::TGeodeticCoords origin(36.8381, -2.4597, 50.0);
const mrpt::topography::TGeodeticCoords pt(36.8390, -2.4580, 55.0);

// Local East-North-Up coordinates (meters) of `pt` with respect to `origin`:
mrpt::math::TPoint3D enu;
mrpt::topography::geodeticToENU_WGS84(pt, enu, origin);

// UTM coordinates:
mrpt::topography::TUTMCoords utm;
int zone = 0;
char band = 0;
mrpt::topography::geodeticToUTM(pt, utm, zone, band);
```

## Examples

- \ref topography_gps_coords_example: GPS coordinates into local ENU and
  geocentric coordinates.
- \ref topography_coordinate_conversion_example: conversions between
  geodetic, geocentric, ENU and UTM coordinates.
- Python: [mrpt_topography_example.py](pymrpt_example_mrpt_topography_example.html).

# Library contents
