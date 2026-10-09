"""
Python bindings for mrpt::maps — metric map representations
"""
from __future__ import annotations
import mrpt.img
import mrpt.math
import mrpt.obs
import mrpt.poses
import mrpt.rtti
import mrpt.serialization
import mrpt.viz
import numpy
import typing
__all__: list[str] = ['CBeacon', 'CBeaconMap', 'CGenericPointsMap', 'CHeightGridMap2D', 'CMetricMap', 'CMultiMetricMap', 'CObservationPointCloud', 'COccupancyGridMap2D', 'COccupancyGridMap3D', 'COctoMap', 'CPointsMap', 'CSimplePointsMap', 'CVoxelMap', 'CVoxelMapRGB', 'PointCloudRecoloringParameters', 'VisualizationParameters', 'obs_to_viz']
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
    def getPoint(self, i: int) -> tuple:
        """
        Returns (x, y, z) tuple for point i
        """
    def getPointsAsNumpy(self) -> numpy.ndarray:
        """
        Returns all points as an Nx3 float32 numpy array
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
    def setPointsFromNumpy(self, arr: numpy.ndarray) -> None:
        """
        Load an Nx3 float32 numpy array into this point cloud
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
class COccupancyGridMap3D(CMetricMap):
    """
    A 3D occupancy grid map with a regular, even distribution of voxels.
    """
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
class CHeightGridMap2D(CMetricMap):
    """
    Digital Elevation Model (DEM), a mesh or grid representation of a surface which keeps the estimated height for each (x,y) location.
    """
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
