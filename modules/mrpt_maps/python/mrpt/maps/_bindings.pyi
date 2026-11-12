"""
Python bindings for mrpt::maps — metric map representations
"""
from __future__ import annotations
import mrpt.config
import mrpt.img
import mrpt.math
import mrpt.obs
import mrpt.poses
import mrpt.rtti
import mrpt.serialization
import mrpt.viz
import numpy
import typing
__all__: list[str] = ['CBeacon', 'CBeaconMap', 'CGenericPointsMap', 'CHeightGridMap2D', 'CMetricMap', 'CMultiMetricMap', 'CObservationPointCloud', 'COccupancyGridMap2D', 'COccupancyGridMap3D', 'COctoMap', 'CPointsMap', 'CSimplePointsMap', 'CVoxelMap', 'CVoxelMapRGB', 'PointCloudRecoloringParameters', 'TVoxelMap_InsertionOptions', 'TVoxelMap_LikelihoodOptions', 'VisualizationParameters', 'obs_to_viz']
class CMetricMap(mrpt.serialization.CSerializable):
    """
    Declares a virtual base class for all metric maps storage classes.
    """
    genericMapParams: mrpt.obs.TMapGenericParams
    def GetRuntimeClass(self) -> mrpt.rtti.TRuntimeClassId:
        """
        Returns information about the class of an object in runtime.
        """
    def __str__(self) -> str:
        ...
    def boundingBox(self) -> mrpt.math.TBoundingBox:
        """
        Bounding box of the map contents
        """
    def canComputeObservationLikelihood(self, obs: mrpt.obs.CObservation) -> bool:
        """
        Returns true if this map is able to compute a sensible likelihood function for this observation (i.e. an occupancy grid map cannot with an image).
        """
    def clear(self) -> None:
        """
        Erase all the contents of the map.
        """
    def computeObservationLikelihood(self, obs: mrpt.obs.CObservation, takenFrom: mrpt.poses.CPose3D) -> float:
        """
        Log-likelihood of an observation taken from a given robot pose
        """
    def computeObservationsLikelihood(self, sf: mrpt.obs.CSensoryFrame, takenFrom: mrpt.poses.CPose3D) -> float:
        """
        Log-likelihood of all observations in a CSensoryFrame taken from a given pose
        """
    def getVisualization(self) -> mrpt.viz.CSetOfObjects:
        """
        Returns a 3D representation of the map as a CSetOfObjects
        """
    def getVisualizationInto(self, outObj: mrpt.viz.CSetOfObjects) -> None:
        """
        Appends a 3D representation of the map to a CSetOfObjects
        """
    def insertObs(self, sf: mrpt.obs.CSensoryFrame, robotPose: mrpt.poses.CPose3D | None = None) -> bool:
        """
        Inserts all the observations of a CSensoryFrame. Returns true if any was inserted.
        """
    def insertObservation(self, obs: mrpt.obs.CObservation, robotPose: mrpt.poses.CPose3D = None) -> bool:
        """
        Insert an observation into the map. Returns true if the map was updated.
        """
    def isEmpty(self) -> bool:
        """
        Returns true if the map is empty/no observation has been inserted.
        """
    def loadFromSimpleMap(self, simpleMap: mrpt.obs.CSimpleMap) -> None:
        """
        Clears the map and builds it from all keyframes of a CSimpleMap
        """
    def saveMetricMapRepresentationToFile(self, filNamePrefix: str) -> None:
        """
        Saves the map in a format suitable for inspection (e.g. images or text)
        """
class CPointsMap(CMetricMap):
    """
    A cloud of points in 2D or 3D, which can be built from a sequence of laser scans or other sensors.
    """
    def __len__(self) -> int:
        ...
    def __repr__(self) -> str:
        ...
    def getLoadPLYErrorString(self) -> str:
        """
        Description of the error of the last failed loadFromPlyFile().
        """
    def getPoint(self, i: int) -> tuple:
        """
        Returns (x, y, z) tuple for point i
        """
    def getPointFieldsAsNumpy(self) -> dict:
        """
        Returns a dict with a 1D NumPy array (a copy) for x, y, z and every other point field, with its dtype: float32, float64, uint16, uint8 or uint32.
        """
    def getPointsAsNumpy(self) -> numpy.ndarray:
        """
        Returns all points as an Nx3 float32 numpy array
        """
    def getSavePLYErrorString(self) -> str:
        """
        Description of the error of the last failed saveToPlyFile().
        """
    def insertPoint(self, x: float, y: float, z: float = 0.0) -> None:
        """
        Provides a way to insert (append) individual points into the map: the missing fields of child classes (color, weight, etc) are left to their default values.
        """
    def isEmpty(self) -> bool:
        """
        Returns true if the map is empty/no observation has been inserted.
        """
    def load2D_from_text_file(self, arg0: str) -> bool:
        """
        Load from a text file. Each line should contain an "X Y" coordinate pair, separated by whitespaces.
        """
    def load3D_from_text_file(self, arg0: str) -> bool:
        """
        Load from a text file. Each line should contain an "X Y Z" coordinate tuple, separated by whitespaces.
        """
    def loadFromKittiVelodyneFile(self, filename: str) -> bool:
        """
        Loads a KITTI Velodyne scan (binary XYZI, gzip-compressed if the name ends in ".gz"), with the intensity in the "intensity" field if the map supports it. Returns True on success.
        """
    def loadFromPlyFile(self, filename: str) -> bool:
        """
        Loads the points from a PLY file. Returns False on error: see getLoadPLYErrorString().
        """
    @typing.overload
    def nn_multiple_search(self, query: mrpt.math.TPoint3D, N: int) -> tuple:
        """
        The N closest points to a 3D query point (KD-tree search), sorted by distance. Returns (indices: uint64 array, points: Nx3 float32 array, squared distances: float32 array).
        """
    @typing.overload
    def nn_multiple_search(self, query: mrpt.math.TPoint2D, N: int) -> tuple:
        """
        The N closest points to a 2D query point, ignoring z (KD-tree search), sorted by distance. Returns (indices: uint64 array, points: Nx2 float32 array, squared distances: float32 array).
        """
    @typing.overload
    def nn_radius_search(self, query: mrpt.math.TPoint3D, search_radius_sqr: float, maxPoints: int = 0) -> tuple:
        """
        The points within a distance of a 3D query point (KD-tree search). The radius is given squared; maxPoints=0 means no limit. Returns (indices: uint64 array, points: Nx3 float32 array, squared distances: float32 array).
        """
    @typing.overload
    def nn_radius_search(self, query: mrpt.math.TPoint2D, search_radius_sqr: float, maxPoints: int = 0) -> tuple:
        """
        The points within a distance of a 2D query point, ignoring z (KD-tree search). The radius is given squared; maxPoints=0 means no limit. Returns (indices: uint64 array, points: Nx2 float32 array, squared distances: float32 array).
        """
    @typing.overload
    def nn_single_search(self, query: mrpt.math.TPoint3D) -> typing.Any:
        """
        Closest point to a 3D query point (KD-tree search). Returns (index, point, squared distance), or None if none was found.
        """
    @typing.overload
    def nn_single_search(self, query: mrpt.math.TPoint2D) -> typing.Any:
        """
        Closest point to a 2D query point, ignoring z (KD-tree search). Returns (index, point, squared distance), or None if none was found.
        """
    def reserve(self, arg0: int) -> None:
        """
        Reserves memory for a given number of points, without changing the map size.
        """
    def save2D_to_text_file(self, arg0: str) -> bool:
        """
        Save to a text file. Each line will contain "X Y" point coordinates.
        """
    def save3D_to_text_file(self, arg0: str) -> bool:
        """
        Save to a text file. Each line will contain "X Y Z" point coordinates.
        """
    def saveToKittiVelodyneFile(self, filename: str) -> bool:
        """
        Saves the points and their "intensity" field in the binary XYZI format of the KITTI dataset, always compressed: use a file name ending in ".gz" to load it back. Returns True on success.
        """
    def saveToPlyFile(self, filename: str, save_in_binary: bool = False, file_comments: list[str] = []) -> bool:
        """
        Saves the points to a PLY file (text or binary). Returns False if the file cannot be created, or on other errors: see getSavePLYErrorString().
        """
    def setPointsFromNumpy(self, arr: numpy.ndarray) -> None:
        """
        Replaces all the points with those of an Nx3 array (other point fields are reset to their default values).
        """
    def size(self) -> int:
        """
        Returns the number of points.
        """
class CSimplePointsMap(CPointsMap):
    """
    A cloud of points in 2D or 3D, which can be built from a sequence of laser scans.
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def __repr__(self) -> str:
        ...
class CGenericPointsMap(CPointsMap):
    """
    A map of 3D points (X,Y,Z) plus any number of custom, string-keyed per-point data channels.
    """
    @staticmethod
    def FromNumpyFields(fields: dict) -> CGenericPointsMap:
        """
        Builds a map from a dict of 1D arrays of the same length, as returned by getPointFieldsAsNumpy(): x, y and z (converted to float32) and other fields, registered with the type given by their dtype (float32, float64, uint16, uint8 or uint32).
        """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def __repr__(self) -> str:
        ...
    def getPointFieldNames_double(self) -> list[str]:
        """
        Get list of all double channel names.
        """
    def getPointFieldNames_float(self) -> list[str]:
        """
        Get list of all float channel names.
        """
    def getPointFieldNames_uint16(self) -> list[str]:
        """
        Get list of all uint16_t channel names.
        """
    def getPointFieldNames_uint32(self) -> list[str]:
        """
        List all uint32 channel names (New in MRPT 3.0.0)
        """
    def getPointFieldNames_uint8(self) -> list[str]:
        """
        Get list of all uint8_t channel names.
        """
    def getPointField_double(self, index: int, fieldName: str) -> float:
        """
        Read the value of a double channel for a given point. Returns 0 if field does not exist.
        """
    def getPointField_float(self, index: int, fieldName: str) -> float:
        """
        Read the value of a float channel for a given point. Returns 0 if field does not exist.
        """
    def getPointField_uint16(self, index: int, fieldName: str) -> int:
        """
        Read the value of a uint16_t channel for a given point. Returns 0 if field does not exist.
        """
    def getPointField_uint32(self, index: int, fieldName: str) -> int:
        """
        Read a uint32 channel value (New in MRPT 3.0.0)
        """
    def getPointField_uint8(self, index: int, fieldName: str) -> int:
        """
        Read the value of a uint8_t channel for a given point. Returns 0 if field does not exist.
        """
    def hasPointField(self, fieldName: str) -> bool:
        """
        Returns true if the map has a data channel with the given name.
        """
    def registerField_double(self, fieldName: str) -> bool:
        """
        Register a new per-point data channel of type float64
        """
    def registerField_float(self, fieldName: str) -> bool:
        """
        Register a new per-point data channel of type float32
        """
    def registerField_uint16(self, fieldName: str) -> bool:
        """
        Register a new per-point data channel of type uint16
        """
    def registerField_uint32(self, fieldName: str) -> bool:
        """
        Register a new per-point data channel of type uint32 (New in MRPT 3.0.0)
        """
    def registerField_uint8(self, fieldName: str) -> bool:
        """
        Register a new per-point data channel of type uint8
        """
    def resize(self, newLength: int) -> None:
        """
        Resizes all point buffers so they can hold the given number of points: newly created points are set to default values, and old contents are not changed.
        """
    def setPointField_double(self, index: int, fieldName: str, value: float) -> None:
        """
        Sets the value of a double channel for a given point.
        """
    def setPointField_float(self, index: int, fieldName: str, value: float) -> None:
        """
        Sets the value of a float channel for a given point.
        """
    def setPointField_uint16(self, index: int, fieldName: str, value: int) -> None:
        """
        Sets the value of a uint16_t channel for a given point.
        """
    def setPointField_uint32(self, index: int, fieldName: str, value: int) -> None:
        """
        Set a uint32 channel value (New in MRPT 3.0.0)
        """
    def setPointField_uint8(self, index: int, fieldName: str, value: int) -> None:
        """
        Sets the value of a uint8_t channel for a given point.
        """
    def unregisterField(self, fieldName: str) -> bool:
        """
        Removes a data channel; returns True if it existed
        """
class COccupancyGridMap2D(CMetricMap):
    """
    A 2D occupancy grid map: each cell holds its probability of being occupied.
    """
    class TInsertionOptions(mrpt.config.CLoadableOptions):
        """
        Options for inserting observations into a COccupancyGridMap2D.
        """
        def __init__(self) -> None:
            """
            Default constructor.
            """
        @property
        def CFD_features_gaussian_size(self) -> float:
            """
            Sigma of the Gaussian filter of getAsImageFiltered() (default: 1, 0: disabled).
            """
        @CFD_features_gaussian_size.setter
        def CFD_features_gaussian_size(self, arg0: float) -> None:
            ...
        @property
        def CFD_features_median_size(self) -> float:
            """
            Size of the median filter of getAsImageFiltered() (default: 3, 0: disabled).
            """
        @CFD_features_median_size.setter
        def CFD_features_median_size(self, arg0: float) -> None:
            ...
        @property
        def considerInvalidRangesAsFreeSpace(self) -> bool:
            """
            Use invalid ranges (no echo) as free space, if the previous and next rays are also invalid (default: true).
            """
        @considerInvalidRangesAsFreeSpace.setter
        def considerInvalidRangesAsFreeSpace(self, arg0: bool) -> None:
            ...
        @property
        def decimation(self) -> int:
            """
            Decimation of range scans (default: 1, all rays).
            """
        @decimation.setter
        def decimation(self, arg0: int) -> None:
            ...
        @property
        def horizontalTolerance(self) -> float:
            """
            Tolerance in pitch and roll for a scan to be horizontal, in radians.
            """
        @horizontalTolerance.setter
        def horizontalTolerance(self, arg0: float) -> None:
            ...
        @property
        def mapAltitude(self) -> float:
            """
            Altitude (z) of the 2D scans to insert (0.01 m tolerance), if useMapAltitude is true.
            """
        @mapAltitude.setter
        def mapAltitude(self, arg0: float) -> None:
            ...
        @property
        def maxDistanceInsertion(self) -> float:
            """
            Largest distance at which cells are updated (default: 15 m).
            """
        @maxDistanceInsertion.setter
        def maxDistanceInsertion(self, arg0: float) -> None:
            ...
        @property
        def maxFreenessInvalidRanges(self) -> float:
            """
            Like maxFreenessUpdateCertainty, for invalid ranges (default: 0, the same as maxOccupancyUpdateCertainty).
            """
        @maxFreenessInvalidRanges.setter
        def maxFreenessInvalidRanges(self, arg0: float) -> None:
            ...
        @property
        def maxFreenessUpdateCertainty(self) -> float:
            """
            Value in [0.5, 1] for the update of free cells (default: 0, the same as maxOccupancyUpdateCertainty).
            """
        @maxFreenessUpdateCertainty.setter
        def maxFreenessUpdateCertainty(self, arg0: float) -> None:
            ...
        @property
        def maxOccupancyUpdateCertainty(self) -> float:
            """
            Value in [0.5, 1] for the Bayesian update of occupied cells (default: 0.65).
            """
        @maxOccupancyUpdateCertainty.setter
        def maxOccupancyUpdateCertainty(self, arg0: float) -> None:
            ...
        @property
        def raytraceStepSizeInCellUnits(self) -> float:
            """
            Unused: kept for backwards compatibility.
            """
        @raytraceStepSizeInCellUnits.setter
        def raytraceStepSizeInCellUnits(self, arg0: float) -> None:
            ...
        @property
        def useMapAltitude(self) -> bool:
            """
            Only insert 2D scans at mapAltitude (default: false).
            """
        @useMapAltitude.setter
        def useMapAltitude(self, arg0: bool) -> None:
            ...
        @property
        def wideningBeamsWithDistance(self) -> bool:
            """
            Rays widen with distance, like real laser beams (default: false).
            """
        @wideningBeamsWithDistance.setter
        def wideningBeamsWithDistance(self, arg0: bool) -> None:
            ...
    class TLikelihoodMethod:
        """
        Method to compute the likelihood of observations.
        
        Members:
        
          lmMeanInformation
        
          lmRayTracing
        
          lmConsensus
        
          lmCellsDifference
        
          lmLikelihoodField_Thrun
        
          lmLikelihoodField_II
        
          lmConsensusOWA
        """
        __members__: typing.ClassVar[dict[str, COccupancyGridMap2D.TLikelihoodMethod]]
        lmCellsDifference: typing.ClassVar[COccupancyGridMap2D.TLikelihoodMethod]
        lmConsensus: typing.ClassVar[COccupancyGridMap2D.TLikelihoodMethod]
        lmConsensusOWA: typing.ClassVar[COccupancyGridMap2D.TLikelihoodMethod]
        lmLikelihoodField_II: typing.ClassVar[COccupancyGridMap2D.TLikelihoodMethod]
        lmLikelihoodField_Thrun: typing.ClassVar[COccupancyGridMap2D.TLikelihoodMethod]
        lmMeanInformation: typing.ClassVar[COccupancyGridMap2D.TLikelihoodMethod]
        lmRayTracing: typing.ClassVar[COccupancyGridMap2D.TLikelihoodMethod]
        def __eq__(self, other: typing.Any) -> bool:
            ...
        def __getstate__(self) -> int:
            ...
        def __hash__(self) -> int:
            ...
        def __index__(self) -> int:
            ...
        def __init__(self, value: int) -> None:
            ...
        def __int__(self) -> int:
            ...
        def __ne__(self, other: typing.Any) -> bool:
            ...
        def __repr__(self) -> str:
            ...
        def __setstate__(self, state: int) -> None:
            ...
        def __str__(self) -> str:
            ...
        @property
        def name(self) -> str:
            ...
        @property
        def value(self) -> int:
            ...
    class TLikelihoodOptions(mrpt.config.CLoadableOptions):
        """
        Options for computing the likelihood of observations in a COccupancyGridMap2D.
        """
        def __init__(self) -> None:
            """
            Default constructor.
            """
        @property
        def LF_alternateAverageMethod(self) -> bool:
            """
            [LikelihoodField] Average the likelihood of the ranges instead of multiplying them (default: false).
            """
        @LF_alternateAverageMethod.setter
        def LF_alternateAverageMethod(self, arg0: bool) -> None:
            ...
        @property
        def LF_decimation(self) -> int:
            """
            [LikelihoodField] Decimation of the points of a scan (default: 5, 1: none).
            """
        @LF_decimation.setter
        def LF_decimation(self, arg0: int) -> None:
            ...
        @property
        def LF_maxCorrsDistance(self) -> float:
            """
            [LikelihoodField] Maximum distance to search correspondences of each point.
            """
        @LF_maxCorrsDistance.setter
        def LF_maxCorrsDistance(self, arg0: float) -> None:
            ...
        @property
        def LF_maxRange(self) -> float:
            """
            [LikelihoodField] Maximum range of the sensor (default: 81 m).
            """
        @LF_maxRange.setter
        def LF_maxRange(self, arg0: float) -> None:
            ...
        @property
        def LF_stdHit(self) -> float:
            """
            [LikelihoodField] Range sigma (default: 0.35).
            """
        @LF_stdHit.setter
        def LF_stdHit(self, arg0: float) -> None:
            ...
        @property
        def LF_useSquareDist(self) -> bool:
            """
            [LikelihoodField] Use exp(-0.5*d^4/s^2) instead of exp(-0.5*d^2/s^2) (default: false).
            """
        @LF_useSquareDist.setter
        def LF_useSquareDist(self, arg0: bool) -> None:
            ...
        @property
        def LF_zHit(self) -> float:
            """
            [LikelihoodField] Weight of hits (default: 0.95).
            """
        @LF_zHit.setter
        def LF_zHit(self, arg0: float) -> None:
            ...
        @property
        def LF_zRandom(self) -> float:
            """
            [LikelihoodField] Weight of random measurements (default: 0.05).
            """
        @LF_zRandom.setter
        def LF_zRandom(self, arg0: float) -> None:
            ...
        @property
        def MI_exponent(self) -> float:
            """
            [MI] Exponent (default: 2.5).
            """
        @MI_exponent.setter
        def MI_exponent(self, arg0: float) -> None:
            ...
        @property
        def MI_ratio_max_distance(self) -> float:
            """
            [MI] Ratio of the maximum distance, with respect to the insertion one (default: 1.5).
            """
        @MI_ratio_max_distance.setter
        def MI_ratio_max_distance(self, arg0: float) -> None:
            ...
        @property
        def MI_skip_rays(self) -> int:
            """
            [MI] Use one out of N rays (default: 10).
            """
        @MI_skip_rays.setter
        def MI_skip_rays(self, arg0: int) -> None:
            ...
        @property
        def OWA_weights(self) -> list[float]:
            """
            [OWA] Weights of the sorted likelihood values (the largest first). Assign a whole list to change it.
            """
        @OWA_weights.setter
        def OWA_weights(self, arg0: list[float]) -> None:
            ...
        @property
        def consensus_pow(self) -> float:
            """
            [Consensus] Power of the likelihood (default: 5).
            """
        @consensus_pow.setter
        def consensus_pow(self, arg0: float) -> None:
            ...
        @property
        def consensus_takeEachRange(self) -> int:
            """
            [Consensus] Use one out of N ranges (default: 1).
            """
        @consensus_takeEachRange.setter
        def consensus_takeEachRange(self, arg0: int) -> None:
            ...
        @property
        def enableLikelihoodCache(self) -> bool:
            """
            Use a cache of likelihood values, for the likelihood field methods (default: true).
            """
        @enableLikelihoodCache.setter
        def enableLikelihoodCache(self, arg0: bool) -> None:
            ...
        @property
        def likelihoodMethod(self) -> COccupancyGridMap2D.TLikelihoodMethod:
            """
            Likelihood method (default: lmLikelihoodField_Thrun).
            """
        @likelihoodMethod.setter
        def likelihoodMethod(self, arg0: COccupancyGridMap2D.TLikelihoodMethod) -> None:
            ...
        @property
        def rayTracing_decimation(self) -> int:
            """
            [rayTracing] Simulate one out of N rays (default: 10).
            """
        @rayTracing_decimation.setter
        def rayTracing_decimation(self, arg0: int) -> None:
            ...
        @property
        def rayTracing_stdHit(self) -> float:
            """
            [rayTracing] Range sigma (default: 1).
            """
        @rayTracing_stdHit.setter
        def rayTracing_stdHit(self, arg0: float) -> None:
            ...
        @property
        def rayTracing_useDistanceFilter(self) -> bool:
            """
            [rayTracing] Unused: kept for backwards compatibility.
            """
        @rayTracing_useDistanceFilter.setter
        def rayTracing_useDistanceFilter(self, arg0: bool) -> None:
            ...
    lmCellsDifference: typing.ClassVar[COccupancyGridMap2D.TLikelihoodMethod]
    lmConsensus: typing.ClassVar[COccupancyGridMap2D.TLikelihoodMethod]
    lmConsensusOWA: typing.ClassVar[COccupancyGridMap2D.TLikelihoodMethod]
    lmLikelihoodField_II: typing.ClassVar[COccupancyGridMap2D.TLikelihoodMethod]
    lmLikelihoodField_Thrun: typing.ClassVar[COccupancyGridMap2D.TLikelihoodMethod]
    lmMeanInformation: typing.ClassVar[COccupancyGridMap2D.TLikelihoodMethod]
    lmRayTracing: typing.ClassVar[COccupancyGridMap2D.TLikelihoodMethod]
    def __init__(self, xMin: float = -10.0, xMax: float = 10.0, yMin: float = -10.0, yMax: float = 10.0, resolution: float = 0.10000000149011612) -> None:
        """
        Constructor.
        """
    def __repr__(self) -> str:
        ...
    def getAsNumpy(self) -> numpy.ndarray:
        """
        Returns the occupancy grid as an HxW float32 numpy array (0=occupied, 1=free)
        """
    def getCell(self, x: int, y: int) -> float:
        """
        Get occupancy probability [0,1] at cell (x,y)
        """
    def getPos(self, x: float, y: float) -> float:
        """
        Get occupancy probability at metric position (x,y)
        """
    def getResolution(self) -> float:
        """
        Returns the resolution of the grid map.
        """
    def getSizeX(self) -> int:
        """
        Returns the horizontal size of grid map in cells count.
        """
    def getSizeY(self) -> int:
        """
        Returns the vertical size of grid map in cells count.
        """
    def getXMax(self) -> float:
        """
        Returns the "x" coordinate of right side of grid map.
        """
    def getXMin(self) -> float:
        """
        Returns the "x" coordinate of left side of grid map.
        """
    def getYMax(self) -> float:
        """
        Returns the "y" coordinate of bottom side of grid map.
        """
    def getYMin(self) -> float:
        """
        Returns the "y" coordinate of top side of grid map.
        """
    def idx2x(self, arg0: int) -> float:
        """
        Transform a cell index into a coordinate value (center of the cell)
        """
    def idx2y(self, arg0: int) -> float:
        """
        Transforms a cell index into a y coordinate (center of the cell).
        """
    def isEmpty(self) -> bool:
        """
        Returns true upon map construction or after calling clear(), the return changes to false upon successful insertObservation() or any other method to load data in the map.
        """
    def laserScanSimulator(self, inout_Scan: mrpt.obs.CObservation2DRangeScan, robotPose: mrpt.poses.CPose2D, threshold: float = 0.6000000238418579, N: int = 361, noiseStd: float = 0.0, decimation: int = 1, angleNoiseStd: float = 0.0) -> None:
        """
        Simulates a 2D laser scan from robotPose (CPose2D) into this grid. inout_Scan (CObservation2DRangeScan) must have its aperture, maxRange, sensorPose and rightToLeft set, and gets the N simulated ranges. threshold is the minimum occupancy of a cell to be an obstacle; noiseStd and angleNoiseStd are the sigmas of the range and angle noise; only one ray out of 'decimation' is simulated.
        """
    def loadFromBitmapFile(self, file: str, resolution: float) -> bool:
        """
        Loads the grid map from an image file, given its resolution and origin.
        """
    def loadFromROSMapServerYAML(self, yamlFilePath: str) -> bool:
        """
        Load a ROS map_server YAML + PNG/PGM file pair
        """
    def saveAsBitmapFile(self, arg0: str) -> bool:
        """
        Saves the grid map as an image file; the format is given by the file extension.
        """
    def setCell(self, x: int, y: int, value: float) -> None:
        """
        Set occupancy probability [0,1] at cell (x,y)
        """
    def setFromNumpy(self, array: numpy.ndarray) -> None:
        """
        Sets all the cells from an HxW array, with the same layout and values as getAsNumpy() (0=occupied, 1=free).
        """
    def setPos(self, x: float, y: float, value: float) -> None:
        """
        Set occupancy probability at metric position (x,y)
        """
    def x2idx(self, x: float) -> int:
        """
        Transform a coordinate value into a cell index. Uses floor() to correctly handle negative coordinates near zero.
        """
    def y2idx(self, y: float) -> int:
        """
        Transforms a y coordinate into a cell index.
        """
    @property
    def insertionOptions(self) -> COccupancyGridMap2D.TInsertionOptions:
        """
        Options for inserting observations (TInsertionOptions).
        """
    @insertionOptions.setter
    def insertionOptions(self, arg0: COccupancyGridMap2D.TInsertionOptions) -> None:
        ...
    @property
    def likelihoodOptions(self) -> COccupancyGridMap2D.TLikelihoodOptions:
        """
        Options for computing the likelihood of observations (TLikelihoodOptions).
        """
    @likelihoodOptions.setter
    def likelihoodOptions(self, arg0: COccupancyGridMap2D.TLikelihoodOptions) -> None:
        ...
class CMultiMetricMap(CMetricMap):
    """
    A set of metric maps of any type, updated and queried together.
    """
    def __getitem__(self, arg0: int) -> CMetricMap:
        ...
    @typing.overload
    def __init__(self) -> None:
        """
        Default ctor: empty list of maps.
        """
    @typing.overload
    def __init__(self, initializers: mrpt.obs.TSetOfMetricMapInitializers) -> None:
        """
        Creates the maps described by a TSetOfMetricMapInitializers
        """
    def __iter__(self) -> typing.Iterator:
        ...
    def __len__(self) -> int:
        ...
    def __repr__(self) -> str:
        ...
    def __setitem__(self, arg0: int, arg1: CMetricMap) -> None:
        """
        Replaces the i-th map
        """
    def clearMaps(self) -> None:
        """
        Removes all maps (clear() only empties them)
        """
    def mapByIndex(self, index: int) -> CMetricMap:
        """
        Gets the i-th map.
        """
    def push_back(self, map: CMetricMap) -> None:
        """
        Appends a new child map to the list.
        """
    def setListOfMaps(self, initializers: mrpt.obs.TSetOfMetricMapInitializers) -> None:
        """
        Replaces all maps with the ones described by a TSetOfMetricMapInitializers
        """
    def size(self) -> int:
        """
        Number of child maps.
        """
    @property
    def maps(self) -> list[CMetricMap]:
        """
        A list with all the maps (to replace one, use map[i] = newMap)
        """
class CVoxelMap(CMetricMap):
    """
    A sparse 3D occupancy voxel map, with log-odds occupancy per voxel.
    """
    def __init__(self, resolution: float = 0.05, inner_bits: int = 2, leaf_bits: int = 3) -> None:
        """
        Creates an empty map with the given voxel size (meters).
        """
    def __repr__(self) -> str:
        ...
    def getOccupiedVoxels(self) -> CSimplePointsMap:
        """
        Returns the centers of all occupied voxels as a CSimplePointsMap
        """
    def getPointOccupancy(self, x: float, y: float, z: float) -> float | None:
        """
        Occupancy probability [0,1] of the voxel at a point, or None if not observed
        """
    def insertPointCloudAsEndPoints(self, points: CPointsMap, sensorPt: mrpt.math.TPoint3D) -> None:
        """
        Inserts a point cloud updating only the end points
        """
    def insertPointCloudAsRays(self, points: CPointsMap, sensorPt: mrpt.math.TPoint3D) -> None:
        """
        Inserts a point cloud, marking free space along the rays from sensorPt
        """
    def updateVoxel(self, x: float, y: float, z: float, occupied: bool) -> None:
        """
        Updates one voxel with an occupied or free observation
        """
    @property
    def insertionOptions(self) -> TVoxelMap_InsertionOptions:
        """
        Options for inserting observations (TVoxelMap_InsertionOptions).
        """
    @insertionOptions.setter
    def insertionOptions(self, arg0: TVoxelMap_InsertionOptions) -> None:
        ...
    @property
    def likelihoodOptions(self) -> TVoxelMap_LikelihoodOptions:
        """
        Options for computing the likelihood of observations (TVoxelMap_LikelihoodOptions).
        """
    @likelihoodOptions.setter
    def likelihoodOptions(self, arg0: TVoxelMap_LikelihoodOptions) -> None:
        ...
class CVoxelMapRGB(CMetricMap):
    """
    A sparse 3D occupancy voxel map, with log-odds occupancy and an RGB color per voxel.
    """
    def __init__(self, resolution: float = 0.05, inner_bits: int = 2, leaf_bits: int = 3) -> None:
        """
        Creates an empty map with the given voxel size (meters).
        """
    def __repr__(self) -> str:
        ...
    def getOccupiedVoxels(self) -> CSimplePointsMap:
        """
        Returns the centers of all occupied voxels as a CSimplePointsMap
        """
    def getPointOccupancy(self, x: float, y: float, z: float) -> float | None:
        """
        Occupancy probability [0,1] of the voxel at a point, or None if not observed
        """
    def insertPointCloudAsEndPoints(self, points: CPointsMap, sensorPt: mrpt.math.TPoint3D) -> None:
        """
        Inserts a point cloud updating only the end points
        """
    def insertPointCloudAsRays(self, points: CPointsMap, sensorPt: mrpt.math.TPoint3D) -> None:
        """
        Inserts a point cloud, marking free space along the rays from sensorPt
        """
    def updateVoxel(self, x: float, y: float, z: float, occupied: bool) -> None:
        """
        Updates one voxel with an occupied or free observation
        """
    @property
    def insertionOptions(self) -> TVoxelMap_InsertionOptions:
        """
        Options for inserting observations (TVoxelMap_InsertionOptions).
        """
    @insertionOptions.setter
    def insertionOptions(self, arg0: TVoxelMap_InsertionOptions) -> None:
        ...
    @property
    def likelihoodOptions(self) -> TVoxelMap_LikelihoodOptions:
        """
        Options for computing the likelihood of observations (TVoxelMap_LikelihoodOptions).
        """
    @likelihoodOptions.setter
    def likelihoodOptions(self, arg0: TVoxelMap_LikelihoodOptions) -> None:
        ...
class COccupancyGridMap3D(CMetricMap):
    """
    A 3D occupancy grid map with a regular, even distribution of voxels.
    """
    class TInsertionOptions(mrpt.config.CLoadableOptions):
        """
        Options for inserting observations into a COccupancyGridMap3D.
        """
        def __init__(self) -> None:
            """
            Default constructor.
            """
        @property
        def decimation(self) -> int:
            """
            Decimation of point clouds and 2D range scans (default: 1).
            """
        @decimation.setter
        def decimation(self, arg0: int) -> None:
            ...
        @property
        def decimation_3d_range(self) -> int:
            """
            Decimation of 3D range scans: keeps the minimum range of each NxN block (default: 8).
            """
        @decimation_3d_range.setter
        def decimation_3d_range(self, arg0: int) -> None:
            ...
        @property
        def maxDistanceInsertion(self) -> float:
            """
            Largest distance at which voxels are updated (default: 15 m).
            """
        @maxDistanceInsertion.setter
        def maxDistanceInsertion(self, arg0: float) -> None:
            ...
        @property
        def maxFreenessUpdateCertainty(self) -> float:
            """
            Value in [0.5, 1] for the update of free voxels (default: 0, the same as maxOccupancyUpdateCertainty).
            """
        @maxFreenessUpdateCertainty.setter
        def maxFreenessUpdateCertainty(self, arg0: float) -> None:
            ...
        @property
        def maxOccupancyUpdateCertainty(self) -> float:
            """
            Value in [0.5, 1] for the Bayesian update of occupied voxels (default: 0.65).
            """
        @maxOccupancyUpdateCertainty.setter
        def maxOccupancyUpdateCertainty(self, arg0: float) -> None:
            ...
        @property
        def raytraceEmptyCells(self) -> bool:
            """
            Mark the voxels along the rays as free (default: true), or only update the end points.
            """
        @raytraceEmptyCells.setter
        def raytraceEmptyCells(self, arg0: bool) -> None:
            ...
    class TLikelihoodMethod:
        """
        Method to compute the likelihood of observations.
        
        Members:
        
          lmLikelihoodField_Thrun
        
          lmRayTracing
        """
        __members__: typing.ClassVar[dict[str, COccupancyGridMap3D.TLikelihoodMethod]]
        lmLikelihoodField_Thrun: typing.ClassVar[COccupancyGridMap3D.TLikelihoodMethod]
        lmRayTracing: typing.ClassVar[COccupancyGridMap3D.TLikelihoodMethod]
        def __eq__(self, other: typing.Any) -> bool:
            ...
        def __getstate__(self) -> int:
            ...
        def __hash__(self) -> int:
            ...
        def __index__(self) -> int:
            ...
        def __init__(self, value: int) -> None:
            ...
        def __int__(self) -> int:
            ...
        def __ne__(self, other: typing.Any) -> bool:
            ...
        def __repr__(self) -> str:
            ...
        def __setstate__(self, state: int) -> None:
            ...
        def __str__(self) -> str:
            ...
        @property
        def name(self) -> str:
            ...
        @property
        def value(self) -> int:
            ...
    class TLikelihoodOptions(mrpt.config.CLoadableOptions):
        """
        Options for computing the likelihood of observations in a COccupancyGridMap3D (not implemented yet for this map: likelihoods are always 0).
        """
        def __init__(self) -> None:
            """
            Default constructor.
            """
        @property
        def LF_decimation(self) -> int:
            """
            [LikelihoodField] Decimation of the points (default: 1, none).
            """
        @LF_decimation.setter
        def LF_decimation(self, arg0: int) -> None:
            ...
        @property
        def LF_maxCorrsDistance(self) -> float:
            """
            [LikelihoodField] Maximum distance to search correspondences of each point.
            """
        @LF_maxCorrsDistance.setter
        def LF_maxCorrsDistance(self, arg0: float) -> None:
            ...
        @property
        def LF_maxRange(self) -> float:
            """
            [LikelihoodField] Maximum range of the sensor (default: 20 m).
            """
        @LF_maxRange.setter
        def LF_maxRange(self, arg0: float) -> None:
            ...
        @property
        def LF_stdHit(self) -> float:
            """
            [LikelihoodField] Range sigma (default: 0.35).
            """
        @LF_stdHit.setter
        def LF_stdHit(self, arg0: float) -> None:
            ...
        @property
        def LF_useSquareDist(self) -> bool:
            """
            [LikelihoodField] Use exp(-0.5*d^4/s^2) instead of exp(-0.5*d^2/s^2) (default: false).
            """
        @LF_useSquareDist.setter
        def LF_useSquareDist(self, arg0: bool) -> None:
            ...
        @property
        def LF_zHit(self) -> float:
            """
            [LikelihoodField] Weight of hits (default: 0.95).
            """
        @LF_zHit.setter
        def LF_zHit(self, arg0: float) -> None:
            ...
        @property
        def LF_zRandom(self) -> float:
            """
            [LikelihoodField] Weight of random measurements (default: 0.05).
            """
        @LF_zRandom.setter
        def LF_zRandom(self, arg0: float) -> None:
            ...
        @property
        def likelihoodMethod(self) -> COccupancyGridMap3D.TLikelihoodMethod:
            """
            Likelihood method (default: lmLikelihoodField_Thrun).
            """
        @likelihoodMethod.setter
        def likelihoodMethod(self, arg0: COccupancyGridMap3D.TLikelihoodMethod) -> None:
            ...
        @property
        def rayTracing_decimation(self) -> int:
            """
            [rayTracing] Simulate one out of N rays (default: 10).
            """
        @rayTracing_decimation.setter
        def rayTracing_decimation(self, arg0: int) -> None:
            ...
        @property
        def rayTracing_stdHit(self) -> float:
            """
            [rayTracing] Range sigma (default: 1).
            """
        @rayTracing_stdHit.setter
        def rayTracing_stdHit(self, arg0: float) -> None:
            ...
    lmLikelihoodField_Thrun: typing.ClassVar[COccupancyGridMap3D.TLikelihoodMethod]
    lmRayTracing: typing.ClassVar[COccupancyGridMap3D.TLikelihoodMethod]
    def __init__(self, corner_min: mrpt.math.TPoint3D = ..., corner_max: mrpt.math.TPoint3D = ..., resolution: float = 0.25) -> None:
        """
        Constructor.
        """
    def __repr__(self) -> str:
        ...
    def fill(self, default_value: float = 0.5) -> None:
        """
        Sets all voxels to a freeness value
        """
    def getCellFreeness(self, cx: int, cy: int, cz: int) -> float:
        """
        Freeness probability [0,1] of a voxel by index (1 = free)
        """
    def getFreenessByPos(self, x: float, y: float, z: float) -> float:
        """
        Freeness probability [0,1] at a metric position (1 = free)
        """
    def getOccupiedVoxelsAsNumpy(self, occupiedThreshold: float = 0.5) -> numpy.ndarray:
        """
        Returns the centers of the voxels with an occupancy probability above the threshold, as an Nx3 float32 array.
        """
    def getResolution(self) -> float:
        """
        Voxel size (meters)
        """
    def getSizeX(self) -> int:
        """
        Number of voxels in X
        """
    def getSizeY(self) -> int:
        """
        Number of voxels in Y
        """
    def getSizeZ(self) -> int:
        """
        Number of voxels in Z
        """
    def setCellFreeness(self, cx: int, cy: int, cz: int, value: float) -> None:
        """
        Sets the freeness probability [0,1] of a voxel by index
        """
    def setFreenessByPos(self, x: float, y: float, z: float, value: float) -> None:
        """
        Sets the freeness probability [0,1] at a metric position
        """
    @property
    def insertionOptions(self) -> COccupancyGridMap3D.TInsertionOptions:
        """
        Options for inserting observations (TInsertionOptions).
        """
    @insertionOptions.setter
    def insertionOptions(self, arg0: COccupancyGridMap3D.TInsertionOptions) -> None:
        ...
    @property
    def likelihoodOptions(self) -> COccupancyGridMap3D.TLikelihoodOptions:
        """
        Options for computing the likelihood of observations (TLikelihoodOptions).
        """
    @likelihoodOptions.setter
    def likelihoodOptions(self, arg0: COccupancyGridMap3D.TLikelihoodOptions) -> None:
        ...
class CHeightGridMap2D(CMetricMap):
    """
    Digital Elevation Model (DEM), a mesh or grid representation of a surface which keeps the estimated height for each (x,y) location.
    """
    class TInsertionOptions(mrpt.config.CLoadableOptions):
        """
        Options for inserting observations into a CHeightGridMap2D.
        """
        def __init__(self) -> None:
            """
            Default constructor.
            """
        @property
        def colorMap(self) -> mrpt.img.TColormap:
            """
            Color map for the 3D view (default: cmJET).
            """
        @colorMap.setter
        def colorMap(self, arg0: mrpt.img.TColormap) -> None:
            ...
        @property
        def filterByHeight(self) -> bool:
            """
            Only insert points with z in [z_min, z_max], relative to the robot (default: false).
            """
        @filterByHeight.setter
        def filterByHeight(self, arg0: bool) -> None:
            ...
        @property
        def z_max(self) -> float:
            """
            Maximum z, if filterByHeight is true (default: 0.5).
            """
        @z_max.setter
        def z_max(self, arg0: float) -> None:
            ...
        @property
        def z_min(self) -> float:
            """
            Minimum z, if filterByHeight is true (default: -0.5).
            """
        @z_min.setter
        def z_min(self, arg0: float) -> None:
            ...
    def __init__(self, xMin: float = -2.0, xMax: float = 2.0, yMin: float = -2.0, yMax: float = 2.0, resolution: float = 0.1) -> None:
        """
        Creates a height map with the given limits and resolution.
        """
    def countObservedCells(self) -> int:
        """
        Return the number of cells with at least one height data inserted.
        """
    def getAsNumpy(self) -> numpy.ndarray:
        """
        Returns the heights as an HxW float64 array (NaN for unobserved cells)
        """
    def getHeight(self, x: float, y: float) -> float | None:
        """
        Height at a metric position, or None if not observed
        """
    def getResolution(self) -> float:
        """
        Returns the resolution of the grid map.
        """
    def getSizeX(self) -> int:
        """
        Returns the horizontal size of grid map in cells count.
        """
    def getSizeY(self) -> int:
        """
        Returns the vertical size of grid map in cells count.
        """
    def getXMin(self) -> float:
        """
        Returns the "x" coordinate of left side of grid map.
        """
    def getYMin(self) -> float:
        """
        Returns the "y" coordinate of top side of grid map.
        """
    def insertIndividualPoint(self, x: float, y: float, z: float) -> bool:
        """
        Inserts one (x,y,z) point. Returns False if out of the map.
        """
    @property
    def insertionOptions(self) -> CHeightGridMap2D.TInsertionOptions:
        """
        Options for inserting observations (TInsertionOptions).
        """
    @insertionOptions.setter
    def insertionOptions(self, arg0: CHeightGridMap2D.TInsertionOptions) -> None:
        ...
class CBeacon(mrpt.serialization.CSerializable):
    """
    The class for storing individual "beacon landmarks" under a variety of 3D position PDF distributions.
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def getMean(self) -> mrpt.poses.CPoint3D:
        """
        Mean position of the beacon
        """
    @property
    def m_ID(self) -> int:
        """
        Beacon ID
        """
    @m_ID.setter
    def m_ID(self, arg0: int) -> None:
        ...
class CBeaconMap(CMetricMap):
    """
    A class for storing a map of 3D probabilistic beacons, using a Montecarlo, Gaussian, or Sum of Gaussians (SOG) representation (for range-only SLAM).
    """
    def __getitem__(self, arg0: int) -> CBeacon:
        ...
    def __init__(self) -> None:
        """
        Constructor.
        """
    def __len__(self) -> int:
        ...
    def push_back(self, beacon: CBeacon) -> None:
        """
        Appends a beacon to the map.
        """
    def size(self) -> int:
        """
        Returns the stored landmarks count.
        """
class COctoMap(CMetricMap):
    """
    A three-dimensional probabilistic occupancy grid, implemented as an octo-tree with the "octomap" C++ library.
    """
    class TInsertionOptions(mrpt.config.CLoadableOptions):
        """
        Options for inserting observations into a COctoMap. Access them through COctoMap.insertionOptions: the sensor model parameters are stored in the map itself.
        """
        @property
        def clampingThresMax(self) -> float:
            """
            Maximum occupancy probability for clamping (default: 0.971).
            """
        @clampingThresMax.setter
        def clampingThresMax(self, arg1: float) -> None:
            ...
        @property
        def clampingThresMin(self) -> float:
            """
            Minimum occupancy probability for clamping (default: 0.1192).
            """
        @clampingThresMin.setter
        def clampingThresMin(self, arg1: float) -> None:
            ...
        @property
        def maxrange(self) -> float:
            """
            Maximum range of the inserted rays (default: -1, the complete rays).
            """
        @maxrange.setter
        def maxrange(self, arg0: float) -> None:
            ...
        @property
        def occupancyThres(self) -> float:
            """
            Occupancy probability threshold of the sensor model (default: 0.5).
            """
        @occupancyThres.setter
        def occupancyThres(self, arg1: float) -> None:
            ...
        @property
        def probHit(self) -> float:
            """
            Probability of a hit in the sensor model (default: 0.7).
            """
        @probHit.setter
        def probHit(self, arg1: float) -> None:
            ...
        @property
        def probMiss(self) -> float:
            """
            Probability of a miss in the sensor model (default: 0.4).
            """
        @probMiss.setter
        def probMiss(self, arg1: float) -> None:
            ...
        @property
        def pruning(self) -> bool:
            """
            Prune the tree after each insertion (default: true).
            """
        @pruning.setter
        def pruning(self, arg0: bool) -> None:
            ...
    class TLikelihoodOptions(mrpt.config.CLoadableOptions):
        """
        Options for computing the likelihood of observations in a COctoMap.
        """
        def __init__(self) -> None:
            """
            Default constructor.
            """
        @property
        def decimation(self) -> int:
            """
            Use one out of N rays (default: 1, all of them).
            """
        @decimation.setter
        def decimation(self, arg0: int) -> None:
            ...
    def __init__(self, resolution: float = 0.1) -> None:
        """
        Default constructor.
        """
    def getMetricMax(self) -> mrpt.math.TPoint3D:
        """
        Maximum value of the bounding box of all known space in x, y, z.
        """
    def getMetricMin(self) -> mrpt.math.TPoint3D:
        """
        Minimum value of the bounding box of all known space in x, y, z.
        """
    def getOccupiedVoxelsAsNumpy(self) -> tuple:
        """
        Returns the occupied leaf voxels (occupancy >= 0.5) as a tuple (centers: Nx3 float32 array, side lengths: float32 array). Pruned leaves may be larger than the resolution.
        """
    def getPointOccupancy(self, x: float, y: float, z: float) -> float | None:
        """
        Occupancy probability [0,1] at a point, or None if the point is not in the octree
        """
    def getResolution(self) -> float:
        """
        Returns the size of the octomap leaf voxels.
        """
    def insertPointCloud(self, points: CPointsMap, sensor_x: float, sensor_y: float, sensor_z: float) -> None:
        """
        Inserts a point cloud as rays from the sensor position
        """
    def isPointWithinOctoMap(self, x: float, y: float, z: float) -> bool:
        """
        Check whether the given point lies within the volume covered by the octomap (that is, whether it is "mapped")
        """
    def size(self) -> int:
        """
        Number of octree nodes
        """
    def updateVoxel(self, x: float, y: float, z: float, occupied: bool) -> None:
        """
        Updates one voxel with an occupied or free observation
        """
    @property
    def insertionOptions(self) -> COctoMap.TInsertionOptions:
        """
        Options for inserting observations (TInsertionOptions): modify its fields in place.
        """
    @property
    def likelihoodOptions(self) -> COctoMap.TLikelihoodOptions:
        """
        Options for computing the likelihood of observations (TLikelihoodOptions).
        """
    @likelihoodOptions.setter
    def likelihoodOptions(self, arg0: COctoMap.TLikelihoodOptions) -> None:
        ...
class CObservationPointCloud(mrpt.obs.CObservation):
    """
    An observation from any sensor that can be summarized as a pointcloud.
    """
    sensorPose: mrpt.poses.CPose3D
    @typing.overload
    def __init__(self) -> None:
        """
        Default constructor.
        """
    @typing.overload
    def __init__(self, scan: mrpt.obs.CObservation3DRangeScan) -> None:
        """
        Builds a point cloud observation from the 3D points of a depth scan
        """
    def __repr__(self) -> str:
        ...
    def getExternalStorageFile(self) -> str:
        """
        Returns the external file name of the point cloud, if any.
        """
    def isExternallyStored(self) -> bool:
        """
        Returns true if the point cloud is stored in an external file.
        """
    @property
    def pointcloud(self) -> CPointsMap:
        """
        The point cloud (a CPointsMap)
        """
    @pointcloud.setter
    def pointcloud(self, arg0: CPointsMap) -> None:
        ...
class PointCloudRecoloringParameters:
    """
    Parameters for recolorize3Dpc(), or part of VisualizationParameters if using obs_to_viz()
    """
    colorMap: mrpt.img.TColormap
    colorMapMaxCoord: float | None
    colorMapMinCoord: float | None
    colorizeByField: str
    invertColorMapping: bool
    outlierRejectionPercentile: float | None
    def __init__(self) -> None:
        """
        Default constructor.
        """
class VisualizationParameters:
    """
    Here we can customize the way observations will be rendered as 3D objects in obs_to_viz(), obs3Dscan_to_viz(), etc.
    """
    axisLimits: float
    axisTickFrequency: float
    axisTickTextSize: float
    colorFromRGBimage: bool
    coloring: PointCloudRecoloringParameters
    drawSensorPose: bool
    onlyPointsWithColor: bool
    pointSize: float
    points2DscansColor: mrpt.img.TColor
    sensorPoseScale: float
    showAxis: bool
    showPointsIn2Dscans: bool
    showSurfaceIn2Dscans: bool
    surface2DscansColor: mrpt.img.TColor
    def __init__(self) -> None:
        """
        Default constructor.
        """
class TVoxelMap_InsertionOptions(mrpt.config.CLoadableOptions):
    """
    Options for inserting observations into a CVoxelMap or CVoxelMapRGB.
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    @property
    def clamp_max(self) -> float:
        """
        Maximum occupancy probability (default: 0.95).
        """
    @clamp_max.setter
    def clamp_max(self, arg0: float) -> None:
        ...
    @property
    def clamp_min(self) -> float:
        """
        Minimum occupancy probability (default: 0.10).
        """
    @clamp_min.setter
    def clamp_min(self, arg0: float) -> None:
        ...
    @property
    def decimation(self) -> int:
        """
        Use one out of N points (default: 1).
        """
    @decimation.setter
    def decimation(self, arg0: int) -> None:
        ...
    @property
    def max_range(self) -> float:
        """
        Maximum range of the inserted rays (<0: no limit).
        """
    @max_range.setter
    def max_range(self, arg0: float) -> None:
        ...
    @property
    def prob_hit(self) -> float:
        """
        Occupancy probability of a voxel where a ray ends, > 0.5 (default: 0.65).
        """
    @prob_hit.setter
    def prob_hit(self, arg0: float) -> None:
        ...
    @property
    def prob_miss(self) -> float:
        """
        Occupancy probability of a voxel crossed by a ray, < 0.5 (default: 0.45).
        """
    @prob_miss.setter
    def prob_miss(self, arg0: float) -> None:
        ...
    @property
    def ray_trace_free_space(self) -> bool:
        """
        Mark the voxels along the rays as free (default: true).
        """
    @ray_trace_free_space.setter
    def ray_trace_free_space(self, arg0: bool) -> None:
        ...
    @property
    def remove_voxels_farther_than(self) -> float:
        """
        If not 0, remove the voxels farther than this L1 distance, in meters (default: 0).
        """
    @remove_voxels_farther_than.setter
    def remove_voxels_farther_than(self, arg0: float) -> None:
        ...
class TVoxelMap_LikelihoodOptions(mrpt.config.CLoadableOptions):
    """
    Options for computing the likelihood of observations in a CVoxelMap or CVoxelMapRGB.
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    @property
    def decimate_up_to(self) -> int:
        """
        Use at most this number of points (default: 0, values <= 1 mean all of them).
        """
    @decimate_up_to.setter
    def decimate_up_to(self, arg0: int) -> None:
        ...
    @property
    def occupiedThreshold(self) -> float:
        """
        Minimum occupancy for a voxel to be occupied (default: 0.60).
        """
    @occupiedThreshold.setter
    def occupiedThreshold(self, arg0: float) -> None:
        ...
@typing.overload
def obs_to_viz(obs: mrpt.obs.CObservation, params: VisualizationParameters = ..., out: mrpt.viz.CSetOfObjects = None) -> mrpt.viz.CSetOfObjects:
    """
    Renders an observation into a CSetOfObjects (a new one if out is None), and returns it
    """
@typing.overload
def obs_to_viz(sf: mrpt.obs.CSensoryFrame, params: VisualizationParameters = ..., out: mrpt.viz.CSetOfObjects = None) -> mrpt.viz.CSetOfObjects:
    """
    Renders all observations of a CSensoryFrame into a CSetOfObjects (a new one if out is None), and returns it
    """
