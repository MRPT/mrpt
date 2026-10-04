"""
mrpt.topography — Geodetic coordinate conversion utilities.

Provides:
  - TCoords           : Degrees/minutes/seconds coordinate type
  - TGeodeticCoords   : (latitude, longitude, height) WGS84 coordinate
  - geodeticToGeocentric_WGS84() : WGS84 geodetic → ECEF geocentric (TPoint3D)
  - geocentricToGeodetic()       : ECEF geocentric → WGS84 geodetic
  - geodeticToENU_WGS84()        : WGS84 geodetic → local ENU coordinates
  - ENUToGeocentric()            : ENU local → ECEF geocentric
  - ENUToGeodetic_WGS84()        : ENU local → WGS84 geodetic
  - TEllipsoid, geodeticToGeocentric() : geodetic ↔ geocentric for any ellipsoid
  - geodeticToUTM(), UTMToGeodetic()   : geodetic ↔ UTM
  - ENU_axes_from_WGS84()        : ENU frame at a given point

Example::

    import mrpt.topography as topo

    origin = topo.TGeodeticCoords(37.0, -7.0, 10.0)  # lat, lon (deg), height (m)
    point  = topo.TGeodeticCoords(37.001, -6.999, 10.0)
    enu = topo.geodeticToENU_WGS84(point, origin)
    print(f"ENU: ({enu.x:.2f}, {enu.y:.2f}) m")
"""
from __future__ import annotations
import mrpt as mrpt
from mrpt.topography._bindings import ENUToGeocentric as ENUToGeocentric
from mrpt.topography._bindings import ENUToGeodetic_WGS84 as ENUToGeodetic_WGS84
from mrpt.topography._bindings import ENU_axes_from_WGS84 as ENU_axes_from_WGS84
from mrpt.topography._bindings import TCoords as TCoords
from mrpt.topography._bindings import TEllipsoid as TEllipsoid
from mrpt.topography._bindings import TGeodeticCoords as TGeodeticCoords
from mrpt.topography._bindings import UTMToGeodetic as UTMToGeodetic
from mrpt.topography._bindings import geocentricToGeodetic as geocentricToGeodetic
from mrpt.topography._bindings import geodeticToENU_WGS84 as geodeticToENU_WGS84
from mrpt.topography._bindings import geodeticToGeocentric as geodeticToGeocentric
from mrpt.topography._bindings import geodeticToGeocentric_WGS84 as geodeticToGeocentric_WGS84
from mrpt.topography._bindings import geodeticToUTM as geodeticToUTM
from . import _bindings
__all__: list = ['TCoords', 'TGeodeticCoords', 'geodeticToGeocentric_WGS84', 'geocentricToGeodetic', 'geodeticToENU_WGS84', 'ENUToGeocentric', 'ENUToGeodetic_WGS84', 'TEllipsoid', 'geodeticToGeocentric', 'geodeticToUTM', 'UTMToGeodetic', 'ENU_axes_from_WGS84']
