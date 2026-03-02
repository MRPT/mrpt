"""
mrpt.topography — Geodetic coordinate conversion utilities.

Provides:
  - TCoords           : Degrees/minutes/seconds coordinate type
  - TGeodeticCoords   : (latitude, longitude, height) WGS84 coordinate
  - geodeticToGeocentric_WGS84() : WGS84 geodetic → ECEF geocentric (TPoint3D)
  - geocentricToGeodetic()       : ECEF geocentric → WGS84 geodetic
  - geodeticToENU_WGS84()        : WGS84 geodetic → local ENU coordinates
  - ENUToGeocentric()            : ENU local → ECEF geocentric

Example::

    import mrpt.topography as topo

    origin = topo.TGeodeticCoords(37.0, -7.0, 10.0)  # lat, lon (deg), height (m)
    point  = topo.TGeodeticCoords(37.001, -6.999, 10.0)
    enu = topo.geodeticToENU_WGS84(point, origin)
    print(f"ENU: ({enu.x:.2f}, {enu.y:.2f}) m")
"""

from mrpt.topography._bindings import (
    ENUToGeocentric,
    TCoords,
    TGeodeticCoords,
    geocentricToGeodetic,
    geodeticToENU_WGS84,
    geodeticToGeocentric_WGS84,
)

__all__ = [
    "TCoords",
    "TGeodeticCoords",
    "geodeticToGeocentric_WGS84",
    "geocentricToGeodetic",
    "geodeticToENU_WGS84",
    "ENUToGeocentric",
]
