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
    genericMapParams: mrpt.obs.TMapGenericParams
    def GetRuntimeClass(self) -> mrpt.rtti.TRuntimeClassId:
        ...
    def __str__(self) -> str:
        ...
    def boundingBox(self) -> mrpt.math.TBoundingBox:
        """
        Bounding box of the map contents
        """
    def canComputeObservationLikelihood(self, obs: mrpt.obs.CObservation) -> bool:
        ...
    def clear(self) -> None:
        ...
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
        ...
    def loadFromSimpleMap(self, simpleMap: mrpt.obs.CSimpleMap) -> None:
        """
        Clears the map and builds it from all keyframes of a CSimpleMap
        """
    def saveMetricMapRepresentationToFile(self, filNamePrefix: str) -> None:
        """
        Saves the map in a format suitable for inspection (e.g. images or text)
        """
class CPointsMap(CMetricMap):
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
        ...
    def isEmpty(self) -> bool:
        ...
    def load2D_from_text_file(self, arg0: str) -> bool:
        ...
    def load3D_from_text_file(self, arg0: str) -> bool:
        ...
    def reserve(self, arg0: int) -> None:
        ...
    def save2D_to_text_file(self, arg0: str) -> bool:
        ...
    def save3D_to_text_file(self, arg0: str) -> bool:
        ...
    def setPointsFromNumpy(self, arr: numpy.ndarray) -> None:
        """
        Load an Nx3 float32 numpy array into this point cloud
        """
    def size(self) -> int:
        ...
class CSimplePointsMap(CPointsMap):
    def __init__(self) -> None:
        ...
    def __repr__(self) -> str:
        ...
class CGenericPointsMap(CPointsMap):
    def __init__(self) -> None:
        ...
    def __repr__(self) -> str:
        ...
    def getPointFieldNames_double(self) -> list[str]:
        ...
    def getPointFieldNames_float(self) -> list[str]:
        ...
    def getPointFieldNames_uint16(self) -> list[str]:
        ...
    def getPointFieldNames_uint32(self) -> list[str]:
        """
        List all uint32 channel names (New in MRPT 3.0.0)
        """
    def getPointFieldNames_uint8(self) -> list[str]:
        ...
    def getPointField_double(self, index: int, fieldName: str) -> float:
        ...
    def getPointField_float(self, index: int, fieldName: str) -> float:
        ...
    def getPointField_uint16(self, index: int, fieldName: str) -> int:
        ...
    def getPointField_uint32(self, index: int, fieldName: str) -> int:
        """
        Read a uint32 channel value (New in MRPT 3.0.0)
        """
    def getPointField_uint8(self, index: int, fieldName: str) -> int:
        ...
    def hasPointField(self, fieldName: str) -> bool:
        ...
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
        ...
    def setPointField_double(self, index: int, fieldName: str, value: float) -> None:
        ...
    def setPointField_float(self, index: int, fieldName: str, value: float) -> None:
        ...
    def setPointField_uint16(self, index: int, fieldName: str, value: int) -> None:
        ...
    def setPointField_uint32(self, index: int, fieldName: str, value: int) -> None:
        """
        Set a uint32 channel value (New in MRPT 3.0.0)
        """
    def setPointField_uint8(self, index: int, fieldName: str, value: int) -> None:
        ...
    def unregisterField(self, fieldName: str) -> bool:
        """
        Removes a data channel; returns True if it existed
        """
class COccupancyGridMap2D(CMetricMap):
    def __init__(self, xMin: float = -10.0, xMax: float = 10.0, yMin: float = -10.0, yMax: float = 10.0, resolution: float = 0.10000000149011612) -> None:
        ...
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
        ...
    def getSizeX(self) -> int:
        ...
    def getSizeY(self) -> int:
        ...
    def getXMax(self) -> float:
        ...
    def getXMin(self) -> float:
        ...
    def getYMax(self) -> float:
        ...
    def getYMin(self) -> float:
        ...
    def idx2x(self, arg0: int) -> float:
        ...
    def idx2y(self, arg0: int) -> float:
        ...
    def isEmpty(self) -> bool:
        ...
    def loadFromBitmapFile(self, file: str, resolution: float) -> bool:
        ...
    def loadFromROSMapServerYAML(self, yamlFilePath: str) -> bool:
        """
        Load a ROS map_server YAML + PNG/PGM file pair
        """
    def saveAsBitmapFile(self, arg0: str) -> bool:
        ...
    def setCell(self, x: int, y: int, value: float) -> None:
        """
        Set occupancy probability [0,1] at cell (x,y)
        """
    def setPos(self, x: float, y: float, value: float) -> None:
        """
        Set occupancy probability at metric position (x,y)
        """
    def x2idx(self, x: float) -> int:
        ...
    def y2idx(self, y: float) -> int:
        ...
class CMultiMetricMap(CMetricMap):
    def __getitem__(self, arg0: int) -> CMetricMap:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
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
        ...
    def push_back(self, map: CMetricMap) -> None:
        ...
    def setListOfMaps(self, initializers: mrpt.obs.TSetOfMetricMapInitializers) -> None:
        """
        Replaces all maps with the ones described by a TSetOfMetricMapInitializers
        """
    def size(self) -> int:
        ...
    @property
    def maps(self) -> list[CMetricMap]:
        """
        A list with all the maps (to replace one, use map[i] = newMap)
        """
class CVoxelMap(CMetricMap):
    def __init__(self, resolution: float = 0.05, inner_bits: int = 2, leaf_bits: int = 3) -> None:
        ...
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
    def __init__(self, resolution: float = 0.05, inner_bits: int = 2, leaf_bits: int = 3) -> None:
        ...
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
    def __init__(self, corner_min: mrpt.math.TPoint3D = ..., corner_max: mrpt.math.TPoint3D = ..., resolution: float = 0.25) -> None:
        ...
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
    def __init__(self, xMin: float = -2.0, xMax: float = 2.0, yMin: float = -2.0, yMax: float = 2.0, resolution: float = 0.1) -> None:
        ...
    def countObservedCells(self) -> int:
        ...
    def getAsNumpy(self) -> numpy.ndarray:
        """
        Returns the heights as an HxW float64 array (NaN for unobserved cells)
        """
    def getHeight(self, x: float, y: float) -> float | None:
        """
        Height at a metric position, or None if not observed
        """
    def getResolution(self) -> float:
        ...
    def getSizeX(self) -> int:
        ...
    def getSizeY(self) -> int:
        ...
    def getXMin(self) -> float:
        ...
    def getYMin(self) -> float:
        ...
    def insertIndividualPoint(self, x: float, y: float, z: float) -> bool:
        """
        Inserts one (x,y,z) point. Returns False if out of the map.
        """
class CBeacon(mrpt.serialization.CSerializable):
    def __init__(self) -> None:
        ...
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
    def __getitem__(self, arg0: int) -> CBeacon:
        ...
    def __init__(self) -> None:
        ...
    def __len__(self) -> int:
        ...
    def push_back(self, beacon: CBeacon) -> None:
        ...
    def size(self) -> int:
        ...
class COctoMap(CMetricMap):
    def __init__(self, resolution: float = 0.1) -> None:
        ...
    def getMetricMax(self) -> mrpt.math.TPoint3D:
        ...
    def getMetricMin(self) -> mrpt.math.TPoint3D:
        ...
    def getPointOccupancy(self, x: float, y: float, z: float) -> float | None:
        """
        Occupancy probability [0,1] at a point, or None if the point is not in the octree
        """
    def getResolution(self) -> float:
        ...
    def insertPointCloud(self, points: CPointsMap, sensor_x: float, sensor_y: float, sensor_z: float) -> None:
        """
        Inserts a point cloud as rays from the sensor position
        """
    def isPointWithinOctoMap(self, x: float, y: float, z: float) -> bool:
        ...
    def size(self) -> int:
        """
        Number of octree nodes
        """
    def updateVoxel(self, x: float, y: float, z: float, occupied: bool) -> None:
        """
        Updates one voxel with an occupied or free observation
        """
class CObservationPointCloud(mrpt.obs.CObservation):
    sensorPose: mrpt.poses.CPose3D
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, scan: mrpt.obs.CObservation3DRangeScan) -> None:
        """
        Builds a point cloud observation from the 3D points of a depth scan
        """
    def __repr__(self) -> str:
        ...
    def getExternalStorageFile(self) -> str:
        ...
    def isExternallyStored(self) -> bool:
        ...
    @property
    def pointcloud(self) -> CPointsMap:
        """
        The point cloud (a CPointsMap)
        """
    @pointcloud.setter
    def pointcloud(self, arg0: CPointsMap) -> None:
        ...
class PointCloudRecoloringParameters:
    colorMap: mrpt.img.TColormap
    colorMapMaxCoord: float | None
    colorMapMinCoord: float | None
    colorizeByField: str
    invertColorMapping: bool
    outlierRejectionPercentile: float | None
    def __init__(self) -> None:
        ...
class VisualizationParameters:
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
