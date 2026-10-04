"""
GNSS message types stored in CObservationGPS
"""
from __future__ import annotations
__all__: list[str] = ['Message_NMEA_GGA', 'Message_NMEA_RMC', 'UTC_time']
class UTC_time:
    hour: int
    minute: int
    sec: float
    def __init__(self) -> None:
        ...
class Message_NMEA_GGA:
    class content_t:
        HDOP: float
        UTCTime: UTC_time
        altitude_meters: float
        corrected_orthometric_altitude: float
        fix_quality: int
        geoidal_distance: float
        latitude_degrees: float
        longitude_degrees: float
        orthometric_altitude: float
        satellitesUsed: int
        thereis_HDOP: bool
        def __init__(self) -> None:
            ...
    fields: Message_NMEA_GGA.content_t
    def __init__(self) -> None:
        ...
class Message_NMEA_RMC:
    class content_t:
        UTCTime: UTC_time
        date_day: int
        date_month: int
        date_year: int
        direction_degrees: float
        latitude_degrees: float
        longitude_degrees: float
        magnetic_dir: float
        positioning_mode: str
        speed_knots: float
        validity_char: int
        def __init__(self) -> None:
            ...
    fields: Message_NMEA_RMC.content_t
    def __init__(self) -> None:
        ...
