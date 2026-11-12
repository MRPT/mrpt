from __future__ import annotations
import mrpt.kinematics
import mrpt.math
import typing
__all__: list[str] = ['CLogFileRecord', 'CParameterizedTrajectoryGenerator', 'TWaypoint', 'TWaypointSequence', 'TWaypointStatus']
class TWaypoint:
    """
    A single navigation waypoint within a TWaypointSequence.
    """
    allow_skip: bool
    allowed_distance: float
    speed_ratio: float
    target: mrpt.math.TPoint2D
    target_frame_id: str
    target_heading: float | None
    @typing.overload
    def __init__(self) -> None:
        """
        Ctor with default values.
        """
    @typing.overload
    def __init__(self, target_x: float, target_y: float, allowed_distance: float, allow_skip: bool = True) -> None:
        """
        Builds a waypoint from its target (x, y), allowed distance and whether it can be skipped.
        """
    def __repr__(self) -> str:
        ...
    def getAsText(self) -> str:
        """
        Get in human-readable format.
        """
    def isValid(self) -> bool:
        """
        Check whether all the minimum mandatory fields have been filled by the user.
        """
class TWaypointSequence:
    """
    A sequence of waypoints for the waypoints navigator.
    """
    waypoints: list[TWaypoint]
    def __getitem__(self, arg0: int) -> TWaypoint:
        ...
    def __init__(self) -> None:
        """
        Ctor with default values.
        """
    def __len__(self) -> int:
        ...
    def __repr__(self) -> str:
        ...
    def append(self, waypoint: TWaypoint) -> None:
        """
        Appends a waypoint to the sequence.
        """
    def clear(self) -> None:
        """
        Removes all waypoints.
        """
    def getAsText(self) -> str:
        """
        Gets navigation params as a human-readable format.
        """
class TWaypointStatus(TWaypoint):
    """
    A TWaypoint augmented with runtime execution status fields.
    """
    reached: bool
    skipped: bool
    def __init__(self) -> None:
        """
        Default constructor.
        """
class CParameterizedTrajectoryGenerator:
    """
    Base class for all Parameterized Trajectory Generators (PTGs).
    """
    @staticmethod
    def CreatePTG(ptg_class_name: str, ini_text: str, section: str, key_prefix: str = '') -> CParameterizedTrajectoryGenerator:
        """
        Factory: create a PTG from INI-format string, section, and key prefix.
        """
    def PTG_IsIntoDomain(self, x: float, y: float) -> bool:
        """
        Returns true if (x, y) is within the PTG domain.
        """
    def alpha2index(self, alpha: float) -> int:
        """
        Discrete path index for the direction alpha (radians).
        """
    def deinitialize(self) -> None:
        """
        De-initializes the PTG, so its parameters can be changed.
        """
    def directionToMotionCommand(self, k: int) -> mrpt.kinematics.CVehicleVelCmd:
        """
        Velocity command (a mrpt.kinematics.CVehicleVelCmd subclass) to follow path k.
        """
    def getAlphaValuesCount(self) -> int:
        """
        Get the number of different, discrete paths in this family.
        """
    def getDescription(self) -> str:
        """
        Gets a short textual description of the PTG and its parameters.
        """
    def getMaxAngVel(self) -> float:
        """
        Maximum angular velocity of this PTG (rad/s).
        """
    def getMaxLinVel(self) -> float:
        """
        Maximum linear velocity of this PTG (m/s).
        """
    def getPathCount(self) -> int:
        """
        Get the number of different, discrete paths in this family.
        """
    def getPathDist(self, k: int, step: int) -> float:
        """
        Distance traveled along path k at a step, in pseudometers (not normalized).
        """
    def getPathPose(self, k: int, step: int) -> mrpt.math.TPose2D:
        """
        Pose of the vehicle (TPose2D) at a step of path k, relative to its start.
        """
    def getPathStepCount(self, k: int) -> int:
        """
        Number of discrete steps of path k.
        """
    def getPathStepDuration(self) -> float:
        """
        Duration of each path step, in seconds.
        """
    def getPathStepForDist(self, k: int, dist: float) -> int | None:
        """
        Largest step of path k with a traveled distance < dist (pseudometers), or None if there is none.
        """
    def getPathTwist(self, k: int, step: int) -> mrpt.math.TTwist2D:
        """
        Velocity (TTwist2D) at a step of path k, in the frame of the start of the path.
        """
    def getRefDistance(self) -> float:
        """
        Reference distance (meters): the length of the paths, used to normalize TP-Space distances.
        """
    def index2alpha(self, k: int) -> float:
        """
        Direction alpha (radians, in [-pi, pi]) of the discrete path index k.
        """
    def initTPObstacleSingle(self, k: int) -> float:
        """
        Returns the initial TP-Obstacle of path k, as initTPObstacles().
        """
    def initTPObstacles(self) -> list[float]:
        """
        Returns the initial TP-Obstacles, one per path: the collision-free distance of each path without obstacles (pseudometers).
        """
    def initialize(self, cacheFilename: str = '', verbose: bool = False) -> None:
        """
        Initializes the PTG; call after setting all its parameters and before using it. PTGs based on collision grids (CPTG_DiffDrive_*) load their grid from cacheFilename, or compute and save it there; an empty name means "cache_<description>.bin.gz" in the current directory. Other PTGs ignore it.
        """
    def inverseMap_WS2TP(self, x: float, y: float, tolerance_dist: float = 0.1) -> tuple[int, float] | None:
        """
        Map a WS point to (k, normalized_d). Returns None if no path found.
        """
    def isInitialized(self) -> bool:
        """
        Returns true if initialize() has been called and there was no errors, so the PTG is ready to be queried for paths, obstacles, etc.
        """
    def loadFromConfigFile(self, ini_text: str, section: str) -> None:
        """
        Loads the PTG parameters from a configuration file section.
        """
    def setRefDistance(self, refDist: float) -> None:
        """
        Sets the reference distance (meters).
        """
    def updateTPObstacle(self, ox: float, oy: float, tp_obstacles: list[float]) -> list[float]:
        """
        Returns tp_obstacles (from initTPObstacles()) updated with an obstacle point (ox, oy), relative to the origin of the PTG: the collision-free distance of each path.
        """
    def updateTPObstacleSingle(self, ox: float, oy: float, k: int, tp_obstacle_k: float) -> float:
        """
        Like updateTPObstacle(), for path k only: returns tp_obstacle_k (from initTPObstacleSingle()) updated with the obstacle point (ox, oy).
        """
class CLogFileRecord:
    """
    One navigation step of the reactive navigator log.
    """
    class TInfoPerPTG:
        """
        Log data of one PTG in one navigation step.
        """
        def __init__(self) -> None:
            """
            Default constructor.
            """
        def __repr__(self) -> str:
            ...
        @property
        def PTG_desc(self) -> str:
            """
            Short description of the PTG
            """
        @PTG_desc.setter
        def PTG_desc(self, arg0: str) -> None:
            ...
        @property
        def TP_Obstacles(self) -> list[float]:
            """
            Distance to obstacles in TP-Space (pseudometers), for directions from -pi to pi
            """
        @TP_Obstacles.setter
        def TP_Obstacles(self, arg1: list[float]) -> None:
            ...
        @property
        def TP_Robot(self) -> mrpt.math.TPoint2D:
            """
            Robot location in TP-Space (normally the origin)
            """
        @TP_Robot.setter
        def TP_Robot(self, arg0: mrpt.math.TPoint2D) -> None:
            ...
        @property
        def TP_Targets(self) -> list[mrpt.math.TPose2D]:
            """
            Target(s) in TP-Space
            """
        @TP_Targets.setter
        def TP_Targets(self, arg0: list[mrpt.math.TPose2D]) -> None:
            ...
        @property
        def desiredDirection(self) -> float:
            """
            Direction chosen by the holonomic method [rad]
            """
        @desiredDirection.setter
        def desiredDirection(self, arg0: float) -> None:
            ...
        @property
        def desiredSpeed(self) -> float:
            """
            Speed chosen by the holonomic method
            """
        @desiredSpeed.setter
        def desiredSpeed(self, arg0: float) -> None:
            ...
        @property
        def evalFactors(self) -> dict[str, float]:
            """
            Evaluation factors, by name
            """
        @evalFactors.setter
        def evalFactors(self, arg0: dict[str, float]) -> None:
            ...
        @property
        def evaluation(self) -> float:
            """
            Final score of this candidate
            """
        @evaluation.setter
        def evaluation(self, arg0: float) -> None:
            ...
        @property
        def timeForHolonomicMethod(self) -> float:
            """
            Time spent in the holonomic method [s]
            """
        @timeForHolonomicMethod.setter
        def timeForHolonomicMethod(self, arg0: float) -> None:
            ...
        @property
        def timeForTPObsTransformation(self) -> float:
            """
            Time to transform obstacles into TP-Space [s]
            """
        @timeForTPObsTransformation.setter
        def timeForTPObsTransformation(self, arg0: float) -> None:
            ...
    WS_targets_relative: list[mrpt.math.TPose2D]
    infoPerPTG: list[CLogFileRecord.TInfoPerPTG]
    nPTGs: int
    relPoseSense: mrpt.math.TPose2D
    relPoseVelCmd: mrpt.math.TPose2D
    robotPoseLocalization: mrpt.math.TPose2D
    robotPoseOdometry: mrpt.math.TPose2D
    def __init__(self) -> None:
        """
        Constructor, builds an empty record.
        """
    def __repr__(self) -> str:
        ...
