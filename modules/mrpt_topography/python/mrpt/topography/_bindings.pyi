"""
Python bindings for mrpt::topography — geodetic coordinate conversions
"""
from __future__ import annotations
import mrpt.math
import typing
__all__: list[str] = ['ENUToGeocentric', 'ENUToGeodetic_WGS84', 'ENU_axes_from_WGS84', 'TCoords', 'TEllipsoid', 'TGeodeticCoords', 'UTMToGeodetic', 'geocentricToGeodetic', 'geodeticToENU_WGS84', 'geodeticToGeocentric', 'geodeticToGeocentric_WGS84', 'geodeticToUTM']
class TCoords:
    """
    An angle in decimal degrees, also settable or readable as degrees, minutes and seconds.
    """
    decimal_value: float
    @typing.overload
    def __init__(self) -> None:
        """
        Default constructor.
        """
    @typing.overload
    def __init__(self, decimal_deg: float) -> None:
        """
        Builds the coordinate from a decimal value in degrees.
        """
    @typing.overload
    def __init__(self, deg: int, min: int, sec: float) -> None:
        """
        Builds the coordinate from degrees, minutes and seconds.
        """
    def __repr__(self) -> str:
        ...
    def getDecimalValue(self) -> float:
        """
        Returns the value in decimal degrees.
        """
    def getDegMinSec(self) -> tuple:
        """
        Returns (degrees, minutes, seconds) tuple
        """
    def setFromDecimal(self, arg0: float) -> None:
        """
        Set from a decimal value (XX.YYYYY) in degrees.
        """
class TGeodeticCoords:
    """
    A set of geodetic coordinates: latitude, longitude and ellipsoidal height, defined over a reference ellipsoid (typically, WGS84)
    """
    @typing.overload
    def __init__(self) -> None:
        """
        Default constructor.
        """
    @typing.overload
    def __init__(self, lat_deg: float, lon_deg: float, height_m: float) -> None:
        """
        Builds the coordinates from latitude, longitude (degrees) and height (meters).
        """
    def __repr__(self) -> str:
        ...
    @property
    def height(self) -> float:
        """
        Geodetic height in meters
        """
    @height.setter
    def height(self, arg0: float) -> None:
        ...
    @property
    def lat(self) -> TCoords:
        """
        Latitude in degrees (TCoords)
        """
    @lat.setter
    def lat(self, arg0: TCoords) -> None:
        ...
    @property
    def lon(self) -> TCoords:
        """
        Longitude in degrees (TCoords)
        """
    @lon.setter
    def lon(self, arg0: TCoords) -> None:
        ...
class TEllipsoid:
    """
    A reference ellipsoid of the Earth, given its semi-major and semi-minor axes.
    """
    name: str
    @staticmethod
    def Ellipsoid_Airy_1830() -> TEllipsoid:
        """
        Returns this reference ellipsoid.
        """
    @staticmethod
    def Ellipsoid_Airy_Modificado_1965() -> TEllipsoid:
        """
        Returns this reference ellipsoid.
        """
    @staticmethod
    def Ellipsoid_Bessel_1841() -> TEllipsoid:
        """
        Returns this reference ellipsoid.
        """
    @staticmethod
    def Ellipsoid_Clarke_1866() -> TEllipsoid:
        """
        Returns this reference ellipsoid.
        """
    @staticmethod
    def Ellipsoid_Clarke_1880() -> TEllipsoid:
        """
        Returns this reference ellipsoid.
        """
    @staticmethod
    def Ellipsoid_Fischer_1960() -> TEllipsoid:
        """
        Returns this reference ellipsoid.
        """
    @staticmethod
    def Ellipsoid_Fischer_1968() -> TEllipsoid:
        """
        Returns this reference ellipsoid.
        """
    @staticmethod
    def Ellipsoid_GRS80() -> TEllipsoid:
        """
        Returns this reference ellipsoid.
        """
    @staticmethod
    def Ellipsoid_Hayford_1909() -> TEllipsoid:
        """
        Returns this reference ellipsoid.
        """
    @staticmethod
    def Ellipsoid_Helmert_1906() -> TEllipsoid:
        """
        Returns this reference ellipsoid.
        """
    @staticmethod
    def Ellipsoid_Hough_1960() -> TEllipsoid:
        """
        Returns this reference ellipsoid.
        """
    @staticmethod
    def Ellipsoid_Internacional_1909() -> TEllipsoid:
        """
        Returns this reference ellipsoid.
        """
    @staticmethod
    def Ellipsoid_Internacional_1924() -> TEllipsoid:
        """
        Returns this reference ellipsoid.
        """
    @staticmethod
    def Ellipsoid_Krasovsky_1940() -> TEllipsoid:
        """
        Returns this reference ellipsoid.
        """
    @staticmethod
    def Ellipsoid_Mercury_1960() -> TEllipsoid:
        """
        Returns this reference ellipsoid.
        """
    @staticmethod
    def Ellipsoid_Mercury_Modificado_1968() -> TEllipsoid:
        """
        Returns this reference ellipsoid.
        """
    @staticmethod
    def Ellipsoid_Nuevo_Internacional_1967() -> TEllipsoid:
        """
        Returns this reference ellipsoid.
        """
    @staticmethod
    def Ellipsoid_Sudamericano_1969() -> TEllipsoid:
        """
        Returns this reference ellipsoid.
        """
    @staticmethod
    def Ellipsoid_WGS66() -> TEllipsoid:
        """
        Returns this reference ellipsoid.
        """
    @staticmethod
    def Ellipsoid_WGS72() -> TEllipsoid:
        """
        Returns this reference ellipsoid.
        """
    @staticmethod
    def Ellipsoid_WGS84() -> TEllipsoid:
        """
        Returns this reference ellipsoid.
        """
    @staticmethod
    def Ellipsoid_Walbeck_1817() -> TEllipsoid:
        """
        Returns this reference ellipsoid.
        """
    @typing.overload
    def __init__(self) -> None:
        """
        Default constructor.
        """
    @typing.overload
    def __init__(self, sa: float, sb: float, name: str) -> None:
        """
        Builds an ellipsoid from its semi-major and semi-minor axes (meters) and name.
        """
    def __repr__(self) -> str:
        ...
    @property
    def sa(self) -> float:
        """
        Largest semiaxis (meters)
        """
    @sa.setter
    def sa(self, arg0: float) -> None:
        ...
    @property
    def sb(self) -> float:
        """
        Smallest semiaxis (meters)
        """
    @sb.setter
    def sb(self, arg0: float) -> None:
        ...
def ENUToGeocentric(enu: mrpt.math.TPoint3D, origin: TGeodeticCoords) -> mrpt.math.TPoint3D:
    """
    Convert ENU local coordinates to ECEF geocentric, given a WGS84 reference origin
    """
def ENUToGeodetic_WGS84(enu: mrpt.math.TPoint3D, origin: TGeodeticCoords) -> TGeodeticCoords:
    """
    Convert local ENU coordinates (meters) relative to origin to a WGS84 geodetic point. Exact inverse of geodeticToENU_WGS84()
    """
def ENU_axes_from_WGS84(coords: TGeodeticCoords, only_angles: bool = False) -> mrpt.math.TPose3D:
    """
    Returns the East-North-Up frame at the given point, as a TPose3D in ECEF coordinates
    """
def UTMToGeodetic(utm: mrpt.math.TPoint3D, zone: int, hemisphere: str | None = None, ellipsoid: TEllipsoid = ..., band: str | None = None) -> TGeodeticCoords:
    """
    Convert UTM coordinates (utm.z is the height) to geodetic coordinates. Give either the hemisphere ('N' or 'S') or the latitude band returned by geodeticToUTM() (band=...).
    """
def geocentricToGeodetic(geocentric: mrpt.math.TPoint3D, ellipsoid: TEllipsoid = ...) -> TGeodeticCoords:
    """
    Convert geocentric ECEF TPoint3D (x,y,z) to geodetic coordinates (default: WGS84)
    """
def geodeticToENU_WGS84(point: TGeodeticCoords, origin: TGeodeticCoords) -> mrpt.math.TPoint3D:
    """
    Convert WGS84 geodetic point to local ENU coordinates (meters) relative to origin
    """
def geodeticToGeocentric(geodetic: TGeodeticCoords, ellipsoid: TEllipsoid) -> mrpt.math.TPoint3D:
    """
    Convert geodetic (lat,lon,h) to geocentric (ECEF) TPoint3D for the given ellipsoid
    """
def geodeticToGeocentric_WGS84(geodetic: TGeodeticCoords) -> mrpt.math.TPoint3D:
    """
    Convert WGS84 geodetic (lat,lon,h) to geocentric (ECEF) TPoint3D (x,y,z) in meters
    """
def geodeticToUTM(geodetic: TGeodeticCoords, ellipsoid: TEllipsoid = ...) -> tuple:
    """
    Convert geodetic coordinates to UTM. Returns (utm: TPoint3D, zone: int, band: str); utm.z is the height
    """
