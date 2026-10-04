"""
Python bindings for mrpt::obs — sensor observations and actions
"""
from __future__ import annotations
import datetime
import mrpt.config
import mrpt.img
import mrpt.math
import mrpt.poses
import mrpt.rtti
import mrpt.serialization
import numpy
import typing
from . import gnss
__all__: list[str] = ['CAction', 'CActionCollection', 'CActionRobotMovement2D', 'CActionRobotMovement3D', 'CObservation', 'CObservation2DRangeScan', 'CObservation3DRangeScan', 'CObservationGPS', 'CObservationIMU', 'CObservationImage', 'CObservationOdometry', 'CObservationRobotPose', 'CRawlog', 'CSensoryFrame', 'CSimpleMap', 'GnssFixType', 'IMU_ALTITUDE', 'IMU_MAG_X', 'IMU_MAG_Y', 'IMU_MAG_Z', 'IMU_ORI_QUAT_W', 'IMU_ORI_QUAT_X', 'IMU_ORI_QUAT_Y', 'IMU_ORI_QUAT_Z', 'IMU_PITCH', 'IMU_PITCH_VEL', 'IMU_PITCH_VEL_GLOBAL', 'IMU_PRESSURE', 'IMU_ROLL', 'IMU_ROLL_VEL', 'IMU_ROLL_VEL_GLOBAL', 'IMU_TEMPERATURE', 'IMU_WX', 'IMU_WY', 'IMU_WZ', 'IMU_X', 'IMU_X_ACC', 'IMU_X_ACC_GLOBAL', 'IMU_X_VEL', 'IMU_Y', 'IMU_YAW', 'IMU_YAW_VEL', 'IMU_YAW_VEL_GLOBAL', 'IMU_Y_ACC', 'IMU_Y_ACC_GLOBAL', 'IMU_Y_VEL', 'IMU_Z', 'IMU_Z_ACC', 'IMU_Z_ACC_GLOBAL', 'IMU_Z_VEL', 'T3DPointsProjectionParams', 'TIMUDataIndex', 'TMapGenericParams', 'TMetricMapInitializer', 'TSetOfMetricMapInitializers', 'gnss']
class CObservation(mrpt.serialization.CSerializable):
    sensorLabel: str
    timestamp: datetime.timedelta
    def GetRuntimeClass(self) -> mrpt.rtti.TRuntimeClassId:
        ...
    def __repr__(self) -> str:
        ...
    def __str__(self) -> str:
        ...
    def getSensorPose(self) -> mrpt.poses.CPose3D:
        """
        Returns the sensor pose (6D) relative to the robot
        """
    def getTimeStamp(self) -> datetime.timedelta:
        ...
    def load(self) -> None:
        """
        Loads externally-stored data (e.g. images), if any
        """
    def unload(self) -> None:
        """
        Frees externally-stored data from memory, if any
        """
class CObservation2DRangeScan(CObservation):
    def __init__(self) -> None:
        ...
    def __repr__(self) -> str:
        ...
    def getScanRange(self, arg0: int) -> float:
        ...
    def getScanRangeValidity(self, arg0: int) -> bool:
        ...
    def getScanRangesAsNumpy(self) -> numpy.ndarray:
        """
        Returns all scan ranges as a 1D float32 numpy array
        """
    def getScanSize(self) -> int:
        ...
    def getValidRangesAsNumpy(self) -> numpy.ndarray:
        """
        Returns validity flags as a 1D bool numpy array
        """
    def resizeScan(self, arg0: int) -> None:
        ...
    def setScanRange(self, arg0: int, arg1: float) -> None:
        ...
    def setScanRangeValidity(self, arg0: int, arg1: bool) -> None:
        ...
    @property
    def aperture(self) -> float:
        """
        Field-of-view in radians
        """
    @aperture.setter
    def aperture(self, arg0: float) -> None:
        ...
    @property
    def maxRange(self) -> float:
        """
        Maximum sensor range in meters
        """
    @maxRange.setter
    def maxRange(self, arg0: float) -> None:
        ...
    @property
    def rightToLeft(self) -> bool:
        """
        Scan direction: True=CCW, False=CW
        """
    @rightToLeft.setter
    def rightToLeft(self, arg0: bool) -> None:
        ...
    @property
    def sensorPose(self) -> mrpt.poses.CPose3D:
        """
        Sensor 6D pose relative to robot base
        """
    @sensorPose.setter
    def sensorPose(self, arg0: mrpt.poses.CPose3D) -> None:
        ...
class CObservationImage(CObservation):
    cameraParams: mrpt.img.TCamera
    cameraPose: mrpt.poses.CPose3D
    image: mrpt.img.CImage
    def __init__(self) -> None:
        ...
    def __repr__(self) -> str:
        ...
class CObservationIMU(CObservation):
    def __init__(self) -> None:
        ...
    def __repr__(self) -> str:
        ...
    def get(self, arg0: typing.Any) -> float:  # unnamed C++ type
        ...
    def getRawMeasurementsAsNumpy(self) -> numpy.ndarray:
        """
        Returns raw IMU measurements as 1D float64 numpy array
        """
    def set(self, arg0: typing.Any, arg1: float) -> None:  # unnamed C++ type
        ...
class TIMUDataIndex:
    """
    Members:
    
      IMU_X_ACC
    
      IMU_Y_ACC
    
      IMU_Z_ACC
    
      IMU_YAW_VEL
    
      IMU_WZ
    
      IMU_PITCH_VEL
    
      IMU_WY
    
      IMU_ROLL_VEL
    
      IMU_WX
    
      IMU_X_VEL
    
      IMU_Y_VEL
    
      IMU_Z_VEL
    
      IMU_YAW
    
      IMU_PITCH
    
      IMU_ROLL
    
      IMU_X
    
      IMU_Y
    
      IMU_Z
    
      IMU_MAG_X
    
      IMU_MAG_Y
    
      IMU_MAG_Z
    
      IMU_PRESSURE
    
      IMU_ALTITUDE
    
      IMU_TEMPERATURE
    
      IMU_ORI_QUAT_X
    
      IMU_ORI_QUAT_Y
    
      IMU_ORI_QUAT_Z
    
      IMU_ORI_QUAT_W
    
      IMU_YAW_VEL_GLOBAL
    
      IMU_PITCH_VEL_GLOBAL
    
      IMU_ROLL_VEL_GLOBAL
    
      IMU_X_ACC_GLOBAL
    
      IMU_Y_ACC_GLOBAL
    
      IMU_Z_ACC_GLOBAL
    """
    IMU_ALTITUDE: typing.ClassVar[TIMUDataIndex]
    IMU_MAG_X: typing.ClassVar[TIMUDataIndex]
    IMU_MAG_Y: typing.ClassVar[TIMUDataIndex]
    IMU_MAG_Z: typing.ClassVar[TIMUDataIndex]
    IMU_ORI_QUAT_W: typing.ClassVar[TIMUDataIndex]
    IMU_ORI_QUAT_X: typing.ClassVar[TIMUDataIndex]
    IMU_ORI_QUAT_Y: typing.ClassVar[TIMUDataIndex]
    IMU_ORI_QUAT_Z: typing.ClassVar[TIMUDataIndex]
    IMU_PITCH: typing.ClassVar[TIMUDataIndex]
    IMU_PITCH_VEL: typing.ClassVar[TIMUDataIndex]
    IMU_PITCH_VEL_GLOBAL: typing.ClassVar[TIMUDataIndex]
    IMU_PRESSURE: typing.ClassVar[TIMUDataIndex]
    IMU_ROLL: typing.ClassVar[TIMUDataIndex]
    IMU_ROLL_VEL: typing.ClassVar[TIMUDataIndex]
    IMU_ROLL_VEL_GLOBAL: typing.ClassVar[TIMUDataIndex]
    IMU_TEMPERATURE: typing.ClassVar[TIMUDataIndex]
    IMU_WX: typing.ClassVar[TIMUDataIndex]
    IMU_WY: typing.ClassVar[TIMUDataIndex]
    IMU_WZ: typing.ClassVar[TIMUDataIndex]
    IMU_X: typing.ClassVar[TIMUDataIndex]
    IMU_X_ACC: typing.ClassVar[TIMUDataIndex]
    IMU_X_ACC_GLOBAL: typing.ClassVar[TIMUDataIndex]
    IMU_X_VEL: typing.ClassVar[TIMUDataIndex]
    IMU_Y: typing.ClassVar[TIMUDataIndex]
    IMU_YAW: typing.ClassVar[TIMUDataIndex]
    IMU_YAW_VEL: typing.ClassVar[TIMUDataIndex]
    IMU_YAW_VEL_GLOBAL: typing.ClassVar[TIMUDataIndex]
    IMU_Y_ACC: typing.ClassVar[TIMUDataIndex]
    IMU_Y_ACC_GLOBAL: typing.ClassVar[TIMUDataIndex]
    IMU_Y_VEL: typing.ClassVar[TIMUDataIndex]
    IMU_Z: typing.ClassVar[TIMUDataIndex]
    IMU_Z_ACC: typing.ClassVar[TIMUDataIndex]
    IMU_Z_ACC_GLOBAL: typing.ClassVar[TIMUDataIndex]
    IMU_Z_VEL: typing.ClassVar[TIMUDataIndex]
    __members__: typing.ClassVar[dict[str, TIMUDataIndex]]
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
class CObservationOdometry(CObservation):
    odometry: mrpt.poses.CPose2D
    def __init__(self) -> None:
        ...
    def __repr__(self) -> str:
        ...
class CObservationRobotPose(CObservation):
    pose: mrpt.poses.CPose3DPDFGaussian
    def __init__(self) -> None:
        ...
    def __repr__(self) -> str:
        ...
class CAction(mrpt.serialization.CSerializable):
    timestamp: datetime.timedelta
    def GetRuntimeClass(self) -> mrpt.rtti.TRuntimeClassId:
        ...
class CActionRobotMovement2D(CAction):
    class TEstimationMethod:
        """
        Members:
        
          emOdometry
        
          emScan2DMatching
        """
        __members__: typing.ClassVar[dict[str, CActionRobotMovement2D.TEstimationMethod]]
        emOdometry: typing.ClassVar[CActionRobotMovement2D.TEstimationMethod]
        emScan2DMatching: typing.ClassVar[CActionRobotMovement2D.TEstimationMethod]
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
    class TDrawSampleMotionModel:
        """
        Members:
        
          mmGaussian
        
          mmThrun
        """
        __members__: typing.ClassVar[dict[str, CActionRobotMovement2D.TDrawSampleMotionModel]]
        mmGaussian: typing.ClassVar[CActionRobotMovement2D.TDrawSampleMotionModel]
        mmThrun: typing.ClassVar[CActionRobotMovement2D.TDrawSampleMotionModel]
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
    class TMotionModelOptions:
        class TOptions_GaussianModel:
            a1: float
            a2: float
            a3: float
            a4: float
            minStdPHI: float
            minStdXY: float
            @typing.overload
            def __init__(self) -> None:
                ...
            @typing.overload
            def __init__(self, a1: float, a2: float, a3: float, a4: float, minStdXY: float, minStdPHI: float) -> None:
                ...
        class TOptions_ThrunModel:
            additional_std_XY: float
            additional_std_phi: float
            alfa1_rot_rot: float
            alfa2_rot_trans: float
            alfa3_trans_trans: float
            alfa4_trans_rot: float
            nParticlesCount: int
            def __init__(self) -> None:
                ...
        gaussianModel: CActionRobotMovement2D.TMotionModelOptions.TOptions_GaussianModel
        modelSelection: CActionRobotMovement2D.TDrawSampleMotionModel
        thrunModel: CActionRobotMovement2D.TMotionModelOptions.TOptions_ThrunModel
        def __init__(self) -> None:
            ...
    emOdometry: typing.ClassVar[CActionRobotMovement2D.TEstimationMethod]
    emScan2DMatching: typing.ClassVar[CActionRobotMovement2D.TEstimationMethod]
    mmGaussian: typing.ClassVar[CActionRobotMovement2D.TDrawSampleMotionModel]
    mmThrun: typing.ClassVar[CActionRobotMovement2D.TDrawSampleMotionModel]
    estimationMethod: CActionRobotMovement2D.TEstimationMethod
    hasVelocities: bool
    motionModelConfiguration: CActionRobotMovement2D.TMotionModelOptions
    velocityLocal: mrpt.math.TTwist2D
    def GetRuntimeClass(self) -> mrpt.rtti.TRuntimeClassId:
        ...
    def __init__(self) -> None:
        ...
    def __repr__(self) -> str:
        ...
    def computeFromOdometry(self, odometryIncrement: mrpt.poses.CPose2D, options: CActionRobotMovement2D.TMotionModelOptions) -> None:
        """
        Computes poseChange from an odometry increment and a motion model
        """
    def drawSingleSample(self) -> mrpt.poses.CPose2D:
        """
        Draws a sample from the motion model
        """
    @property
    def poseChange(self) -> mrpt.poses.CPosePDF:
        """
        The 2D pose change probabilistic estimation (a CPosePDF)
        """
    @poseChange.setter
    def poseChange(self, arg1: mrpt.poses.CPosePDF) -> None:
        ...
    @property
    def rawOdometryIncrementReading(self) -> mrpt.poses.CPose2D:
        """
        Raw odometry reading (increment since last step)
        """
    @rawOdometryIncrementReading.setter
    def rawOdometryIncrementReading(self, arg0: mrpt.poses.CPose2D) -> None:
        ...
class CActionRobotMovement3D(CAction):
    class TMotionModelOptions:
        class TOptions_6DOFModel:
            a1: float
            a10: float
            a2: float
            a3: float
            a4: float
            a5: float
            a6: float
            a7: float
            a8: float
            a9: float
            additional_std_XYZ: float
            additional_std_angle: float
            nParticlesCount: int
            def __init__(self) -> None:
                ...
        mm6DOFModel: CActionRobotMovement3D.TMotionModelOptions.TOptions_6DOFModel
        def __init__(self) -> None:
            ...
    rawOdometryIncrementReading: mrpt.poses.CPose3D
    def __init__(self) -> None:
        ...
    def __repr__(self) -> str:
        ...
    def computeFromOdometry(self, odometryIncrement: mrpt.poses.CPose3D, options: CActionRobotMovement3D.TMotionModelOptions) -> None:
        """
        Computes poseChange from an odometry increment and a motion model
        """
    @property
    def poseChange(self) -> mrpt.poses.CPose3DPDFGaussian:
        """
        Pose change as a CPose3DPDFGaussian
        """
    @poseChange.setter
    def poseChange(self, arg0: mrpt.poses.CPose3DPDFGaussian) -> None:
        ...
class CActionCollection(mrpt.serialization.CSerializable):
    def __init__(self) -> None:
        ...
    def __iter__(self) -> typing.Iterator:
        ...
    def __len__(self) -> int:
        ...
    def clear(self) -> None:
        ...
    def get(self, arg0: int) -> CAction:
        ...
    def getBestMovementEstimation(self) -> CActionRobotMovement2D:
        """
        Returns the CActionRobotMovement2D with the best estimation method, or None
        """
    def insert(self, arg0: CAction) -> None:
        ...
    def size(self) -> int:
        ...
class CSensoryFrame(mrpt.serialization.CSerializable):
    def __getitem__(self, arg0: int) -> CObservation:
        ...
    def __init__(self) -> None:
        ...
    def __iter__(self) -> typing.Iterator:
        ...
    def __len__(self) -> int:
        ...
    def __repr__(self) -> str:
        ...
    def clear(self) -> None:
        ...
    def insert(self, arg0: CObservation) -> None:
        ...
    def size(self) -> int:
        ...
class T3DPointsProjectionParams:
    MAKE_ORGANIZED: bool
    decimation: int
    layer: str
    robotPoseInTheWorld: mrpt.poses.CPose3D | None
    takeIntoAccountSensorPoseOnRobot: bool
    def __init__(self) -> None:
        ...
class CObservation3DRangeScan(CObservation):
    confidenceImage: mrpt.img.CImage
    hasConfidenceImage: bool
    hasIntensityImage: bool
    hasPoints3D: bool
    hasRangeImage: bool
    intensityImage: mrpt.img.CImage
    maxRange: float
    relativePoseIntensityWRTDepth: mrpt.poses.CPose3D
    sensorPose: mrpt.poses.CPose3D
    stdError: float
    def __init__(self) -> None:
        ...
    def __repr__(self) -> str:
        ...
    def getPoints3DAsNumpy(self) -> numpy.ndarray:
        """
        Returns the 3D points as an Nx3 float32 array (see unprojectInto())
        """
    def getRangeImageAsNumpy(self) -> numpy.ndarray:
        """
        Returns the range image as an HxW float32 array, in meters (0 = invalid)
        """
    def getRangeImageRawAsNumpy(self) -> numpy.ndarray:
        """
        Returns the raw range image as an HxW uint16 array (multiply by rangeUnits for meters)
        """
    def getScanSize(self) -> int:
        """
        Number of 3D points (if hasPoints3D)
        """
    def setRangeImageFromNumpy(self, ranges: numpy.ndarray) -> None:
        """
        Sets the range image from an HxW array of ranges in meters. NaN or infinite values are stored as 0 (invalid).
        """
    def unprojectInto(self, params: T3DPointsProjectionParams = ...) -> None:
        """
        Computes the 3D points (points3D_*) from the range image and camera intrinsics
        """
    @property
    def cameraParams(self) -> mrpt.img.TCamera:
        """
        Depth camera intrinsics
        """
    @cameraParams.setter
    def cameraParams(self, arg0: mrpt.img.TCamera) -> None:
        ...
    @property
    def cameraParamsIntensity(self) -> mrpt.img.TCamera:
        """
        Intensity camera intrinsics
        """
    @cameraParamsIntensity.setter
    def cameraParamsIntensity(self, arg0: mrpt.img.TCamera) -> None:
        ...
    @property
    def rangeUnits(self) -> float:
        """
        Meters per unit in the raw range image
        """
    @rangeUnits.setter
    def rangeUnits(self, arg0: float) -> None:
        ...
    @property
    def range_is_depth(self) -> bool:
        """
        True: ranges are depth (Z); False: distances along each pixel ray
        """
    @range_is_depth.setter
    def range_is_depth(self, arg0: bool) -> None:
        ...
class GnssFixType:
    """
    Members:
    
      UNKNOWN
    
      NO_FIX
    
      AUTONOMOUS
    
      SBAS
    
      GBAS
    
      DGPS
    
      RTK_FLOAT
    
      RTK_FIXED
    
      PPP
    
      DEAD_RECKONING
    
      SIMULATION
    """
    AUTONOMOUS: typing.ClassVar[GnssFixType]
    DEAD_RECKONING: typing.ClassVar[GnssFixType]
    DGPS: typing.ClassVar[GnssFixType]
    GBAS: typing.ClassVar[GnssFixType]
    NO_FIX: typing.ClassVar[GnssFixType]
    PPP: typing.ClassVar[GnssFixType]
    RTK_FIXED: typing.ClassVar[GnssFixType]
    RTK_FLOAT: typing.ClassVar[GnssFixType]
    SBAS: typing.ClassVar[GnssFixType]
    SIMULATION: typing.ClassVar[GnssFixType]
    UNKNOWN: typing.ClassVar[GnssFixType]
    __members__: typing.ClassVar[dict[str, GnssFixType]]
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
class CObservationGPS(CObservation):
    fix_type: GnssFixType
    has_satellite_timestamp: bool
    originalReceivedTimestamp: datetime.timedelta
    sensorPose: mrpt.poses.CPose3D
    def __init__(self) -> None:
        ...
    def __repr__(self) -> str:
        ...
    def clear(self) -> None:
        """
        Removes all GNSS messages
        """
    def getGGA(self) -> gnss.Message_NMEA_GGA | None:
        """
        Returns a copy of the NMEA GGA message, or None
        """
    def getRMC(self) -> gnss.Message_NMEA_RMC | None:
        """
        Returns a copy of the NMEA RMC message, or None
        """
    def hasGGA(self) -> bool:
        """
        True if the observation has an NMEA GGA message
        """
    def hasRMC(self) -> bool:
        """
        True if the observation has an NMEA RMC message
        """
    def setGGA(self, msg: gnss.Message_NMEA_GGA) -> None:
        """
        Stores (or replaces) the NMEA GGA message
        """
    def setRMC(self, msg: gnss.Message_NMEA_RMC) -> None:
        """
        Stores (or replaces) the NMEA RMC message
        """
    @property
    def covariance_enu(self) -> mrpt.math.CMatrixDouble33 | None:
        """
        Optional 3x3 ENU position covariance (m^2), or None
        """
    @covariance_enu.setter
    def covariance_enu(self, arg0: mrpt.math.CMatrixDouble33 | None) -> None:
        ...
class TMapGenericParams(mrpt.config.CLoadableOptions, mrpt.serialization.CSerializable):
    enableObservationInsertion: bool
    enableObservationLikelihood: bool
    enableSaveAs3DObject: bool
    def __init__(self) -> None:
        ...
class TMetricMapInitializer(mrpt.config.CLoadableOptions):
    genericMapParams: TMapGenericParams
    @staticmethod
    def factory(mapClassName: str) -> TMetricMapInitializer:
        """
        Creates the definition of a map by its class name, e.g. 'COccupancyGridMap2D'. Requires importing mrpt.maps first, which registers the map types.
        """
    def getMetricMapClassName(self) -> str:
        """
        Returns the C++ class name of the map this definition creates
        """
class TSetOfMetricMapInitializers(mrpt.config.CLoadableOptions):
    def __getitem__(self, arg0: int) -> TMetricMapInitializer:
        ...
    def __init__(self) -> None:
        ...
    def __iter__(self) -> typing.Iterator:
        ...
    def __len__(self) -> int:
        ...
    def __repr__(self) -> str:
        ...
    def clear(self) -> None:
        ...
    def push_back(self, mapDefinition: TMetricMapInitializer) -> None:
        ...
    def size(self) -> int:
        ...
class CSimpleMap(mrpt.serialization.CSerializable):
    class Keyframe:
        localTwist: mrpt.math.TTwist3D | None
        pose: mrpt.poses.CPose3DPDF
        sf: CSensoryFrame
        @typing.overload
        def __init__(self) -> None:
            ...
        @typing.overload
        def __init__(self, pose: mrpt.poses.CPose3DPDF, sf: CSensoryFrame, localTwist: mrpt.math.TTwist3D | None = None) -> None:
            ...
    def __getitem__(self, arg0: int) -> CSimpleMap.Keyframe:
        ...
    def __init__(self) -> None:
        ...
    def __iter__(self) -> typing.Iterator:
        ...
    def __len__(self) -> int:
        ...
    def __repr__(self) -> str:
        ...
    def changeCoordinatesOrigin(self, newOrigin: mrpt.poses.CPose3D) -> None:
        """
        Transforms all keyframe poses so the old origin becomes newOrigin
        """
    def clear(self) -> None:
        ...
    def empty(self) -> bool:
        ...
    def get(self, index: int) -> CSimpleMap.Keyframe:
        ...
    @typing.overload
    def insert(self, pose: mrpt.poses.CPose3DPDF, sf: CSensoryFrame, localTwist: mrpt.math.TTwist3D | None = None) -> None:
        """
        Appends a keyframe
        """
    @typing.overload
    def insert(self, keyframe: CSimpleMap.Keyframe) -> None:
        ...
    def loadFromFile(self, fileName: str) -> bool:
        """
        Loads a .simplemap file (possibly compressed). Returns False on error.
        """
    def remove(self, index: int) -> None:
        ...
    def saveToFile(self, fileName: str) -> bool:
        """
        Saves to a .simplemap file. Returns False on error.
        """
    def size(self) -> int:
        ...
class CRawlog(mrpt.serialization.CSerializable):
    class TEntryType:
        """
        Members:
        
          etSensoryFrame
        
          etActionCollection
        
          etObservation
        
          etOther
        """
        __members__: typing.ClassVar[dict[str, CRawlog.TEntryType]]
        etActionCollection: typing.ClassVar[CRawlog.TEntryType]
        etObservation: typing.ClassVar[CRawlog.TEntryType]
        etOther: typing.ClassVar[CRawlog.TEntryType]
        etSensoryFrame: typing.ClassVar[CRawlog.TEntryType]
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
    etActionCollection: typing.ClassVar[CRawlog.TEntryType]
    etObservation: typing.ClassVar[CRawlog.TEntryType]
    etOther: typing.ClassVar[CRawlog.TEntryType]
    etSensoryFrame: typing.ClassVar[CRawlog.TEntryType]
    @staticmethod
    def ReadFromArchive(archive: mrpt.serialization.CArchive, entry: int = 0) -> tuple:
        """
        Reads the next entry from a rawlog stream (see mrpt.io.archiveFrom()). Returns (readOk, nextEntryIndex, actions, sensoryFrame, observation): either (actions, sensoryFrame) or observation are None, depending on the rawlog format. readOk is False at the end of the stream.
        """
    @staticmethod
    def detectImagesDirectory(rawlogFilename: str) -> str:
        """
        Returns the directory of externally-stored images for a given rawlog file
        """
    def __getitem__(self, arg0: int) -> mrpt.serialization.CSerializable:
        ...
    def __init__(self) -> None:
        ...
    def __iter__(self) -> typing.Iterator:
        """
        Iterates over all entries
        """
    def __len__(self) -> int:
        ...
    def __repr__(self) -> str:
        ...
    def clear(self) -> None:
        ...
    def empty(self) -> bool:
        ...
    def getAsAction(self, index: int) -> CActionCollection:
        """
        Returns entry i as a CActionCollection. Raises if it has a different type.
        """
    def getAsGeneric(self, index: int) -> mrpt.serialization.CSerializable:
        """
        Returns entry i, whatever its type
        """
    def getAsObservation(self, index: int) -> CObservation:
        """
        Returns entry i as a CObservation. Raises if it has a different type.
        """
    def getAsObservations(self, index: int) -> CSensoryFrame:
        """
        Returns entry i as a CSensoryFrame. Raises if it has a different type.
        """
    def getCommentText(self) -> str:
        """
        Returns the embedded comment text, if any
        """
    def getType(self, index: int) -> CRawlog.TEntryType:
        ...
    def insert(self, obj: mrpt.serialization.CSerializable) -> None:
        """
        Appends an object (CSensoryFrame, CActionCollection, CObservation, ...). The object is stored by reference, not copied.
        """
    def loadFromRawLogFile(self, fileName: str, non_obs_objects_are_legal: bool = False) -> bool:
        """
        Loads a .rawlog file (possibly compressed). Returns False on error.
        """
    def remove(self, index: int) -> None:
        ...
    def saveToRawLogFile(self, fileName: str) -> bool:
        """
        Saves to a .rawlog file. Returns False on error.
        """
    def setCommentText(self, text: str) -> None:
        ...
    def size(self) -> int:
        ...
IMU_ALTITUDE: TIMUDataIndex
IMU_MAG_X: TIMUDataIndex
IMU_MAG_Y: TIMUDataIndex
IMU_MAG_Z: TIMUDataIndex
IMU_ORI_QUAT_W: TIMUDataIndex
IMU_ORI_QUAT_X: TIMUDataIndex
IMU_ORI_QUAT_Y: TIMUDataIndex
IMU_ORI_QUAT_Z: TIMUDataIndex
IMU_PITCH: TIMUDataIndex
IMU_PITCH_VEL: TIMUDataIndex
IMU_PITCH_VEL_GLOBAL: TIMUDataIndex
IMU_PRESSURE: TIMUDataIndex
IMU_ROLL: TIMUDataIndex
IMU_ROLL_VEL: TIMUDataIndex
IMU_ROLL_VEL_GLOBAL: TIMUDataIndex
IMU_TEMPERATURE: TIMUDataIndex
IMU_WX: TIMUDataIndex
IMU_WY: TIMUDataIndex
IMU_WZ: TIMUDataIndex
IMU_X: TIMUDataIndex
IMU_X_ACC: TIMUDataIndex
IMU_X_ACC_GLOBAL: TIMUDataIndex
IMU_X_VEL: TIMUDataIndex
IMU_Y: TIMUDataIndex
IMU_YAW: TIMUDataIndex
IMU_YAW_VEL: TIMUDataIndex
IMU_YAW_VEL_GLOBAL: TIMUDataIndex
IMU_Y_ACC: TIMUDataIndex
IMU_Y_ACC_GLOBAL: TIMUDataIndex
IMU_Y_VEL: TIMUDataIndex
IMU_Z: TIMUDataIndex
IMU_Z_ACC: TIMUDataIndex
IMU_Z_ACC_GLOBAL: TIMUDataIndex
IMU_Z_VEL: TIMUDataIndex
