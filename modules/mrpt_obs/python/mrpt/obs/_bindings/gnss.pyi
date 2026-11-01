"""
GNSS message types stored in CObservationGPS
"""
from __future__ import annotations
import datetime
__all__: list[str] = ['Message_NMEA_GGA', 'Message_NMEA_GLL', 'Message_NMEA_GSA', 'Message_NMEA_RMC', 'Message_NMEA_VTG', 'Message_NMEA_ZDA', 'UTC_time']
class UTC_time:
    """
    UTC (Coordinated Universal Time) time-stamp structure for GPS messages.
    """
    hour: int
    minute: int
    sec: float
    def __init__(self) -> None:
        """
        Default constructor.
        """
class Message_NMEA_GGA:
    """
    NMEA datum: GGA.
    """
    class content_t:
        """
        Fields of a GGA message.
        """
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
            """
            Default constructor.
            """
    fields: Message_NMEA_GGA.content_t
    def __init__(self) -> None:
        """
        Default constructor.
        """
class Message_NMEA_RMC:
    """
    NMEA datum: RMC.
    """
    class content_t:
        """
        Fields of an RMC message.
        """
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
            """
            Default constructor.
            """
    fields: Message_NMEA_RMC.content_t
    def __init__(self) -> None:
        """
        Default constructor.
        """
class Message_NMEA_GLL:
    """
    NMEA datum: GLL.
    """
    class content_t:
        """
        Fields of a GLL message.
        """
        UTCTime: UTC_time
        latitude_degrees: float
        longitude_degrees: float
        validity_char: int
        def __init__(self) -> None:
            """
            Default constructor.
            """
    fields: Message_NMEA_GLL.content_t
    def __init__(self) -> None:
        """
        Default constructor.
        """
class Message_NMEA_VTG:
    """
    NMEA datum: VTG.
    """
    class content_t:
        """
        Fields of a VTG message.
        """
        ground_speed_kmh: float
        ground_speed_knots: float
        magnetic_track: float
        true_track: float
        def __init__(self) -> None:
            """
            Default constructor.
            """
    fields: Message_NMEA_VTG.content_t
    def __init__(self) -> None:
        """
        Default constructor.
        """
class Message_NMEA_GSA:
    """
    NMEA datum: GSA (DOP and active satellites).
    """
    class content_t:
        """
        Fields of a GSA message.
        """
        HDOP: float
        PDOP: float
        VDOP: float
        auto_selection_fix: str
        fix_2D_3D: str
        def __init__(self) -> None:
            """
            Default constructor.
            """
        @property
        def PRNs(self) -> list[str]:
            """
            IDs of the satellites used in the solution
            """
    fields: Message_NMEA_GSA.content_t
    def __init__(self) -> None:
        """
        Default constructor.
        """
class Message_NMEA_ZDA:
    """
    NMEA datum: ZDA (UTC date and time).
    """
    class content_t:
        """
        Fields of a ZDA message.
        """
        UTCTime: UTC_time
        date_day: int
        date_month: int
        date_year: int
        def __init__(self) -> None:
            """
            Default constructor.
            """
    fields: Message_NMEA_ZDA.content_t
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def getDateTimeAsTimestamp(self) -> datetime.timedelta:
        """
        The UTC date and time as a timestamp
        """
