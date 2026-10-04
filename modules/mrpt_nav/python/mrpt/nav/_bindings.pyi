from __future__ import annotations
import mrpt.math
import typing
__all__: list[str] = ['CLogFileRecord', 'CParameterizedTrajectoryGenerator', 'TWaypoint', 'TWaypointSequence', 'TWaypointStatus']
class TWaypoint:
    allow_skip: bool
    allowed_distance: float
    speed_ratio: float
    target: mrpt.math.TPoint2D
    target_frame_id: str
    target_heading: float | None
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, target_x: float, target_y: float, allowed_distance: float, allow_skip: bool = True) -> None:
        ...
    def __repr__(self) -> str:
        ...
    def getAsText(self) -> str:
        ...
    def isValid(self) -> bool:
        ...
class TWaypointSequence:
    waypoints: list[TWaypoint]
    def __getitem__(self, arg0: int) -> TWaypoint:
        ...
    def __init__(self) -> None:
        ...
    def __len__(self) -> int:
        ...
    def __repr__(self) -> str:
        ...
    def append(self, waypoint: TWaypoint) -> None:
        ...
    def clear(self) -> None:
        ...
    def getAsText(self) -> str:
        ...
class TWaypointStatus(TWaypoint):
    reached: bool
    skipped: bool
    def __init__(self) -> None:
        ...
class CParameterizedTrajectoryGenerator:
    @staticmethod
    def CreatePTG(ptg_class_name: str, ini_text: str, section: str, key_prefix: str = '') -> CParameterizedTrajectoryGenerator:
        """
        Factory: create a PTG from INI-format string, section, and key prefix.
        """
    def PTG_IsIntoDomain(self, x: float, y: float) -> bool:
        ...
    def deinitialize(self) -> None:
        ...
    def getAlphaValuesCount(self) -> int:
        ...
    def getDescription(self) -> str:
        ...
    def getPathCount(self) -> int:
        ...
    def initialize(self) -> None:
        ...
    def inverseMap_WS2TP(self, x: float, y: float, tolerance_dist: float = 0.1) -> tuple[int, float] | None:
        """
        Map a WS point to (k, normalized_d). Returns None if no path found.
        """
    def isInitialized(self) -> bool:
        ...
    def loadFromConfigFile(self, ini_text: str, section: str) -> None:
        ...
class CLogFileRecord:
    WS_targets_relative: list[mrpt.math.TPose2D]
    infoPerPTG: list[typing.Any]  # unnamed C++ type
    nPTGs: int
    relPoseSense: mrpt.math.TPose2D
    relPoseVelCmd: mrpt.math.TPose2D
    robotPoseLocalization: mrpt.math.TPose2D
    robotPoseOdometry: mrpt.math.TPose2D
    def __init__(self) -> None:
        ...
    def __repr__(self) -> str:
        ...
