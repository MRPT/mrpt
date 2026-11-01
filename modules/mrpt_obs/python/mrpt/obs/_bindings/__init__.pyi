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
__all__: list[str] = ['CAction', 'CActionCollection', 'CActionRobotMovement2D', 'CActionRobotMovement3D', 'CObservation', 'CObservation2DRangeScan', 'CObservation3DRangeScan', 'CObservationGPS', 'CObservationIMU', 'CObservationImage', 'CObservationOdometry', 'CObservationRobotPose', 'CObservationRotatingScan', 'CObservationStereoImages', 'CObservationVelodyneScan', 'CRawlog', 'CSensoryFrame', 'CSimpleMap', 'GnssFixType', 'IMU_ALTITUDE', 'IMU_MAG_X', 'IMU_MAG_Y', 'IMU_MAG_Z', 'IMU_ORI_QUAT_W', 'IMU_ORI_QUAT_X', 'IMU_ORI_QUAT_Y', 'IMU_ORI_QUAT_Z', 'IMU_PITCH', 'IMU_PITCH_VEL', 'IMU_PITCH_VEL_GLOBAL', 'IMU_PRESSURE', 'IMU_ROLL', 'IMU_ROLL_VEL', 'IMU_ROLL_VEL_GLOBAL', 'IMU_TEMPERATURE', 'IMU_WX', 'IMU_WY', 'IMU_WZ', 'IMU_X', 'IMU_X_ACC', 'IMU_X_ACC_GLOBAL', 'IMU_X_VEL', 'IMU_Y', 'IMU_YAW', 'IMU_YAW_VEL', 'IMU_YAW_VEL_GLOBAL', 'IMU_Y_ACC', 'IMU_Y_ACC_GLOBAL', 'IMU_Y_VEL', 'IMU_Z', 'IMU_Z_ACC', 'IMU_Z_ACC_GLOBAL', 'IMU_Z_VEL', 'T3DPointsProjectionParams', 'TIMUDataIndex', 'TMapGenericParams', 'TMetricMapInitializer', 'TSetOfMetricMapInitializers', 'VelodyneCalibration', 'VelodyneGeneratePointCloudParameters', 'gnss']
class CObservation(mrpt.serialization.CSerializable):
    """
    Base class of all sensor observations: a timestamp, a sensor label and data.
    """
    sensorLabel: str
    timestamp: datetime.timedelta
    def GetRuntimeClass(self) -> mrpt.rtti.TRuntimeClassId:
        """
        Returns information about the class of an object in runtime.
        """
    def __repr__(self) -> str:
        ...
    def __str__(self) -> str:
        ...
    def getSensorPose(self) -> mrpt.poses.CPose3D:
        """
        Returns the sensor pose (6D) relative to the robot
        """
    def getTimeStamp(self) -> datetime.timedelta:
        """
        Returns the observation timestamp.
        """
    def load(self) -> None:
        """
        Loads externally-stored data (e.g. images), if any
        """
    def unload(self) -> None:
        """
        Frees externally-stored data from memory, if any
        """
class CObservation2DRangeScan(CObservation):
    """
    A "CObservation"-derived class that represents a 2D range scan measurement (typically from a laser scanner).
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def __repr__(self) -> str:
        ...
    def getScanRange(self, arg0: int) -> float:
        """
        Returns the range of the i-th ray, in meters.
        """
    def getScanRangeValidity(self, arg0: int) -> bool:
        """
        Returns whether the i-th ray has a valid range (false: no echo).
        """
    def getScanRangesAsNumpy(self) -> numpy.ndarray:
        """
        Returns all scan ranges as a 1D float32 numpy array
        """
    def getScanSize(self) -> int:
        """
        Get number of scan rays.
        """
    def getValidRangesAsNumpy(self) -> numpy.ndarray:
        """
        Returns validity flags as a 1D bool numpy array
        """
    def resizeScan(self, arg0: int) -> None:
        """
        Resizes all data vectors to allocate a given number of scan rays.
        """
    def setScanRange(self, arg0: int, arg1: float) -> None:
        """
        Sets the range of the i-th ray, in meters.
        """
    def setScanRangeValidity(self, arg0: int, arg1: bool) -> None:
        """
        Sets whether the i-th ray has a valid range.
        """
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
    """
    An image from a camera, along with its pose on the robot and intrinsic parameters.
    """
    cameraParams: mrpt.img.TCamera
    cameraPose: mrpt.poses.CPose3D
    image: mrpt.img.CImage
    def __init__(self) -> None:
        """
        Constructor.
        """
    def __repr__(self) -> str:
        ...
class CObservationStereoImages(CObservation):
    """
    A pair of images from a stereo camera (left and, optionally, right and disparity), with the camera parameters and poses.
    """
    def __init__(self) -> None:
        """
        Constructor.
        """
    def __repr__(self) -> str:
        ...
    def __str__(self) -> str:
        ...
    def areImagesRectified(self) -> bool:
        """
        Whether the images are rectified, by the MRPT convention: all the distortion parameters of leftCamera are zero.
        """
    def getStereoCameraParams(self) -> mrpt.img.TStereoCamera:
        """
        Returns both cameras parameters and their relative pose as a TStereoCamera.
        """
    def setStereoCameraParams(self, params: mrpt.img.TStereoCamera) -> None:
        """
        Sets both cameras parameters and their relative pose.
        """
    @property
    def cameraPose(self) -> mrpt.poses.CPose3DQuat:
        """
        Pose of the left camera on the robot.
        """
    @cameraPose.setter
    def cameraPose(self, arg0: mrpt.poses.CPose3DQuat) -> None:
        ...
    @property
    def hasImageDisparity(self) -> bool:
        """
        Whether imageDisparity holds an image.
        """
    @hasImageDisparity.setter
    def hasImageDisparity(self, arg0: bool) -> None:
        ...
    @property
    def hasImageRight(self) -> bool:
        """
        Whether imageRight holds an image.
        """
    @hasImageRight.setter
    def hasImageRight(self, arg0: bool) -> None:
        ...
    @property
    def imageDisparity(self) -> mrpt.img.CImage:
        """
        Disparity image (only valid if hasImageDisparity).
        """
    @imageDisparity.setter
    def imageDisparity(self, arg0: mrpt.img.CImage) -> None:
        ...
    @property
    def imageLeft(self) -> mrpt.img.CImage:
        """
        Left camera image.
        """
    @imageLeft.setter
    def imageLeft(self, arg0: mrpt.img.CImage) -> None:
        ...
    @property
    def imageRight(self) -> mrpt.img.CImage:
        """
        Right camera image (only valid if hasImageRight).
        """
    @imageRight.setter
    def imageRight(self, arg0: mrpt.img.CImage) -> None:
        ...
    @property
    def leftCamera(self) -> mrpt.img.TCamera:
        """
        Left camera intrinsic parameters.
        """
    @leftCamera.setter
    def leftCamera(self, arg0: mrpt.img.TCamera) -> None:
        ...
    @property
    def rightCamera(self) -> mrpt.img.TCamera:
        """
        Right camera intrinsic parameters.
        """
    @rightCamera.setter
    def rightCamera(self, arg0: mrpt.img.TCamera) -> None:
        ...
    @property
    def rightCameraPose(self) -> mrpt.poses.CPose3DQuat:
        """
        Pose of the right camera with respect to the left one.
        """
    @rightCameraPose.setter
    def rightCameraPose(self, arg0: mrpt.poses.CPose3DQuat) -> None:
        ...
class TIMUDataIndex:
    """
    Index of each measurement in CObservationIMU
    
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
class CObservationIMU(CObservation):
    """
    Measurements of an inertial measurement unit (IMU): orientation, angular velocity, accelerations, etc.
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def __repr__(self) -> str:
        ...
    def get(self, idx: TIMUDataIndex) -> float:
        """
        Returns one measurement (it must have been set).
        """
    def getRawMeasurementsAsNumpy(self) -> numpy.ndarray:
        """
        Returns raw IMU measurements as 1D float64 numpy array
        """
    def set(self, idx: TIMUDataIndex, value: float) -> None:
        """
        Sets one measurement and marks it as valid.
        """
class CObservationOdometry(CObservation):
    """
    An observation of the current (cumulative) odometry for a wheeled robot.
    """
    odometry: mrpt.poses.CPose2D
    def __init__(self) -> None:
        """
        Default ctor.
        """
    def __repr__(self) -> str:
        ...
class CObservationRobotPose(CObservation):
    """
    An observation providing an alternative robot pose from an external source.
    """
    pose: mrpt.poses.CPose3DPDFGaussian
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def __repr__(self) -> str:
        ...
class CAction(mrpt.serialization.CSerializable):
    """
    Base class of robot actions (e.g. odometry increments), stored in rawlogs.
    """
    timestamp: datetime.timedelta
    def GetRuntimeClass(self) -> mrpt.rtti.TRuntimeClassId:
        """
        Returns information about the class of an object in runtime.
        """
class CActionRobotMovement2D(CAction):
    """
    Represents a probabilistic 2D movement of the robot mobile base.
    """
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
        """
        Options of the probabilistic 2D motion models.
        """
        class TOptions_GaussianModel:
            """
            Parameters of the Gaussian motion model.
            """
            a1: float
            a2: float
            a3: float
            a4: float
            minStdPHI: float
            minStdXY: float
            @typing.overload
            def __init__(self) -> None:
                """
                Default constructor.
                """
            @typing.overload
            def __init__(self, a1: float, a2: float, a3: float, a4: float, minStdXY: float, minStdPHI: float) -> None:
                """
                Builds the Gaussian motion model parameters from their values.
                """
        class TOptions_ThrunModel:
            """
            Parameters of the particle-based motion model (Thrun et al.).
            """
            additional_std_XY: float
            additional_std_phi: float
            alfa1_rot_rot: float
            alfa2_rot_trans: float
            alfa3_trans_trans: float
            alfa4_trans_rot: float
            nParticlesCount: int
            def __init__(self) -> None:
                """
                Default constructor.
                """
        gaussianModel: CActionRobotMovement2D.TMotionModelOptions.TOptions_GaussianModel
        modelSelection: CActionRobotMovement2D.TDrawSampleMotionModel
        thrunModel: CActionRobotMovement2D.TMotionModelOptions.TOptions_ThrunModel
        def __init__(self) -> None:
            """
            Default constructor.
            """
    emOdometry: typing.ClassVar[CActionRobotMovement2D.TEstimationMethod]
    emScan2DMatching: typing.ClassVar[CActionRobotMovement2D.TEstimationMethod]
    mmGaussian: typing.ClassVar[CActionRobotMovement2D.TDrawSampleMotionModel]
    mmThrun: typing.ClassVar[CActionRobotMovement2D.TDrawSampleMotionModel]
    estimationMethod: CActionRobotMovement2D.TEstimationMethod
    hasVelocities: bool
    motionModelConfiguration: CActionRobotMovement2D.TMotionModelOptions
    velocityLocal: mrpt.math.TTwist2D
    def GetRuntimeClass(self) -> mrpt.rtti.TRuntimeClassId:
        """
        Returns information about the class of an object in runtime.
        """
    def __init__(self) -> None:
        """
        Default constructor.
        """
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
    """
    Represents a probabilistic motion increment in SE(3).
    """
    class TMotionModelOptions:
        """
        Options of the probabilistic 3D motion models.
        """
        class TOptions_6DOFModel:
            """
            Parameters of the particle-based 6DOF motion model.
            """
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
                """
                Default constructor.
                """
        mm6DOFModel: CActionRobotMovement3D.TMotionModelOptions.TOptions_6DOFModel
        def __init__(self) -> None:
            """
            Default constructor.
            """
    rawOdometryIncrementReading: mrpt.poses.CPose3D
    def __init__(self) -> None:
        """
        Default constructor.
        """
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
    """
    A collection of robot actions, stored in rawlogs.
    """
    def __init__(self) -> None:
        """
        Ctor.
        """
    def __iter__(self) -> typing.Iterator:
        ...
    def __len__(self) -> int:
        ...
    def clear(self) -> None:
        """
        Erase all actions from the list.
        """
    def get(self, arg0: int) -> CAction:
        """
        Returns the i-th action.
        """
    def getBestMovementEstimation(self) -> CActionRobotMovement2D:
        """
        Returns the CActionRobotMovement2D with the best estimation method, or None
        """
    def insert(self, arg0: CAction) -> None:
        """
        Add a new object to the list, making a deep copy.
        """
    def size(self) -> int:
        """
        Returns the actions count in the collection.
        """
class CSensoryFrame(mrpt.serialization.CSerializable):
    """
    A "sensory frame" is a set of observations taken by the robot approximately at the same time, so they can be considered as a multi-sensor "snapshot" of the environment.
    """
    def __getitem__(self, arg0: int) -> CObservation:
        ...
    def __init__(self) -> None:
        """
        Default ctor.
        """
    def __iter__(self) -> typing.Iterator:
        ...
    def __len__(self) -> int:
        ...
    def __repr__(self) -> str:
        ...
    def clear(self) -> None:
        """
        Clear the container, so it holds no observations.
        """
    def insert(self, arg0: CObservation) -> None:
        """
        Appends an observation.
        """
    def size(self) -> int:
        """
        Returns the number of observations in the list.
        """
class T3DPointsProjectionParams:
    """
    Options of CObservation3DRangeScan.unprojectInto().
    """
    MAKE_ORGANIZED: bool
    decimation: int
    layer: str
    robotPoseInTheWorld: mrpt.poses.CPose3D | None
    takeIntoAccountSensorPoseOnRobot: bool
    def __init__(self) -> None:
        """
        Default constructor.
        """
class CObservation3DRangeScan(CObservation):
    """
    A depth or RGB+D image from a time-of-flight or structured-light sensor.
    """
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
        """
        Default constructor.
        """
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
    """
    This class stores messages from GNSS or GNSS+IMU devices, from consumer-grade inexpensive GPS receivers to Novatel/Topcon/... advanced RTK solutions.
    """
    fix_type: GnssFixType
    has_satellite_timestamp: bool
    originalReceivedTimestamp: datetime.timedelta
    sensorPose: mrpt.poses.CPose3D
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def __repr__(self) -> str:
        ...
    def __str__(self) -> str:
        ...
    def clear(self) -> None:
        """
        Removes all GNSS messages
        """
    def getGGA(self) -> gnss.Message_NMEA_GGA | None:
        """
        Returns a copy of the NMEA GGA message, or None
        """
    def getGLL(self) -> gnss.Message_NMEA_GLL | None:
        """
        Returns a copy of the NMEA GLL message, or None
        """
    def getGSA(self) -> gnss.Message_NMEA_GSA | None:
        """
        Returns a copy of the NMEA GSA message, or None
        """
    def getMessageTypes(self) -> list[str]:
        """
        Names of the GNSS messages in this observation (e.g. NMEA_GGA, NV_OEM6_BESTPOS); str(obs) dumps all of them
        """
    def getRMC(self) -> gnss.Message_NMEA_RMC | None:
        """
        Returns a copy of the NMEA RMC message, or None
        """
    def getVTG(self) -> gnss.Message_NMEA_VTG | None:
        """
        Returns a copy of the NMEA VTG message, or None
        """
    def getZDA(self) -> gnss.Message_NMEA_ZDA | None:
        """
        Returns a copy of the NMEA ZDA message, or None
        """
    def hasGGA(self) -> bool:
        """
        True if the observation has an NMEA GGA message
        """
    def hasGLL(self) -> bool:
        """
        True if the observation has an NMEA GLL message
        """
    def hasGSA(self) -> bool:
        """
        True if the observation has an NMEA GSA message
        """
    def hasRMC(self) -> bool:
        """
        True if the observation has an NMEA RMC message
        """
    def hasVTG(self) -> bool:
        """
        True if the observation has an NMEA VTG message
        """
    def hasZDA(self) -> bool:
        """
        True if the observation has an NMEA ZDA message
        """
    def setGGA(self, msg: gnss.Message_NMEA_GGA) -> None:
        """
        Stores (or replaces) the NMEA GGA message
        """
    def setGLL(self, msg: gnss.Message_NMEA_GLL) -> None:
        """
        Stores (or replaces) the NMEA GLL message
        """
    def setGSA(self, msg: gnss.Message_NMEA_GSA) -> None:
        """
        Stores (or replaces) the NMEA GSA message
        """
    def setRMC(self, msg: gnss.Message_NMEA_RMC) -> None:
        """
        Stores (or replaces) the NMEA RMC message
        """
    def setVTG(self, msg: gnss.Message_NMEA_VTG) -> None:
        """
        Stores (or replaces) the NMEA VTG message
        """
    def setZDA(self, msg: gnss.Message_NMEA_ZDA) -> None:
        """
        Stores (or replaces) the NMEA ZDA message
        """
    @property
    def covariance_enu(self) -> mrpt.math.CMatrixDouble33 | None:
        """
        Optional 3x3 ENU position covariance (m^2), or None
        """
    @covariance_enu.setter
    def covariance_enu(self, arg0: mrpt.math.CMatrixDouble33 | None) -> None:
        ...
class VelodyneCalibration:
    """
    Velodyne calibration data, for CObservationVelodyneScan.
    """
    @staticmethod
    def LoadDefaultCalibration(lidar_model: str) -> VelodyneCalibration:
        """
        Default calibration of a model (VLP16, HDL32, HDL64), or an empty one if unknown.
        """
    def __init__(self) -> None:
        """
        Empty calibration.
        """
    def empty(self) -> bool:
        """
        True if no calibration is loaded.
        """
    def loadFromXMLFile(self, filename: str) -> bool:
        """
        Loads a Velodyne XML calibration file. Returns False on error.
        """
    def loadFromYAMLFile(self, filename: str) -> bool:
        """
        Loads a ROS velodyne_pointcloud YAML calibration file. Returns False on error.
        """
class VelodyneGeneratePointCloudParameters:
    """
    Options of CObservationVelodyneScan.generatePointCloud().
    """
    dualKeepLast: bool
    dualKeepStrongest: bool
    filterOutIsolatedPoints: bool
    generatePerPointAzimuth: bool
    generatePerPointTimestamp: bool
    isolatedPointsFilterDistance: float
    maxAzimuth_deg: float
    maxDistance: float
    minAzimuth_deg: float
    minDistance: float
    def __init__(self) -> None:
        """
        Default parameters.
        """
class CObservationVelodyneScan(CObservation):
    """
    Raw packets of one Velodyne scan. generatePointCloud() decodes them into point_cloud, read with the pointCloud*() methods.
    """
    has_satellite_timestamp: bool
    originalReceivedTimestamp: datetime.timedelta
    def __init__(self) -> None:
        """
        Constructor.
        """
    def __repr__(self) -> str:
        ...
    def generatePointCloud(self, params: VelodyneGeneratePointCloudParameters = ...) -> None:
        """
        Decodes the packets into point_cloud (needs a calibration).
        """
    def packetCount(self) -> int:
        """
        Number of raw data packets.
        """
    def pointCloudAzimuth(self) -> numpy.ndarray:
        """
        Azimuth of each point (rad); only with generatePerPointAzimuth.
        """
    def pointCloudIntensity(self) -> numpy.ndarray:
        """
        Intensity of each generated point (uint8).
        """
    def pointCloudLaserId(self) -> numpy.ndarray:
        """
        Laser (ring) of each generated point.
        """
    def pointCloudTimestamps(self) -> numpy.ndarray:
        """
        Time of each point (UNIX seconds); only with generatePerPointTimestamp.
        """
    def pointCloudXYZ(self) -> numpy.ndarray:
        """
        Generated points as an Nx3 float32 array, in the sensor frame.
        """
    def pointCount(self) -> int:
        """
        Number of points generated by generatePointCloud().
        """
    @property
    def calibration(self) -> VelodyneCalibration:
        """
        Calibration used to decode the packets.
        """
    @calibration.setter
    def calibration(self, arg0: VelodyneCalibration) -> None:
        ...
    @property
    def maxRange(self) -> float:
        """
        Maximum valid range (m).
        """
    @maxRange.setter
    def maxRange(self, arg0: float) -> None:
        ...
    @property
    def minRange(self) -> float:
        """
        Minimum valid range (m).
        """
    @minRange.setter
    def minRange(self, arg0: float) -> None:
        ...
    @property
    def sensorPose(self) -> mrpt.poses.CPose3D:
        """
        Pose of the sensor on the robot.
        """
    @sensorPose.setter
    def sensorPose(self, arg0: mrpt.poses.CPose3D) -> None:
        ...
class CObservationRotatingScan(CObservation):
    """
    A scan of a rotating LiDAR (Velodyne, Ouster, ...) as organized range and intensity images: one row per laser, one column per firing.
    """
    has_satellite_timestamp: bool
    originalReceivedTimestamp: datetime.timedelta
    def __init__(self) -> None:
        """
        Constructor.
        """
    def __repr__(self) -> str:
        ...
    def __str__(self) -> str:
        ...
    def fromGeneric(self, obs: CObservation) -> bool:
        """
        Fills this scan from any supported observation. Returns False if not supported.
        """
    def fromScan2D(self, obs: CObservation2DRangeScan) -> None:
        """
        Fills this scan from a 2D LiDAR scan.
        """
    def fromVelodyne(self, obs: CObservationVelodyneScan) -> None:
        """
        Fills this scan from a Velodyne one (with a calibration).
        """
    @property
    def azimuthSpan(self) -> float:
        """
        Azimuth span of the columns (rad).
        """
    @azimuthSpan.setter
    def azimuthSpan(self, arg0: float) -> None:
        ...
    @property
    def columnCount(self) -> int:
        """
        Number of firings (image columns).
        """
    @columnCount.setter
    def columnCount(self, arg0: int) -> None:
        ...
    @property
    def intensityImage(self) -> numpy.ndarray:
        """
        Intensities (uint8, rows x columns), a copy.
        """
    @property
    def lidarModel(self) -> str:
        """
        LiDAR model name.
        """
    @lidarModel.setter
    def lidarModel(self, arg0: str) -> None:
        ...
    @property
    def maxRange(self) -> float:
        """
        Maximum valid range (m).
        """
    @maxRange.setter
    def maxRange(self, arg0: float) -> None:
        ...
    @property
    def minRange(self) -> float:
        """
        Minimum valid range (m).
        """
    @minRange.setter
    def minRange(self, arg0: float) -> None:
        ...
    @property
    def organizedPoints(self) -> numpy.ndarray:
        """
        Points of each range image cell (float32, rows x columns x 3), in the sensor frame.
        """
    @property
    def rangeImage(self) -> numpy.ndarray:
        """
        Ranges (uint16, rows x columns; multiply by rangeResolution for meters, 0: invalid). A copy: assign a whole array to change it.
        """
    @rangeImage.setter
    def rangeImage(self, arg1: numpy.ndarray) -> None:
        ...
    @property
    def rangeResolution(self) -> float:
        """
        Range (m) of one unit of rangeImage.
        """
    @rangeResolution.setter
    def rangeResolution(self, arg0: float) -> None:
        ...
    @property
    def rowCount(self) -> int:
        """
        Number of lasers (image rows).
        """
    @rowCount.setter
    def rowCount(self, arg0: int) -> None:
        ...
    @property
    def sensorPose(self) -> mrpt.poses.CPose3D:
        """
        Pose of the sensor on the robot.
        """
    @sensorPose.setter
    def sensorPose(self, arg0: mrpt.poses.CPose3D) -> None:
        ...
    @property
    def startAzimuth(self) -> float:
        """
        Azimuth of the first column (rad).
        """
    @startAzimuth.setter
    def startAzimuth(self, arg0: float) -> None:
        ...
    @property
    def sweepDuration(self) -> float:
        """
        Duration of the scan (s).
        """
    @sweepDuration.setter
    def sweepDuration(self, arg0: float) -> None:
        ...
class TMapGenericParams(mrpt.config.CLoadableOptions, mrpt.serialization.CSerializable):
    """
    Parameters common to all metric maps.
    """
    enableObservationInsertion: bool
    enableObservationLikelihood: bool
    enableSaveAs3DObject: bool
    def __init__(self) -> None:
        """
        Default constructor.
        """
class TMetricMapInitializer(mrpt.config.CLoadableOptions):
    """
    Base class of the definitions of one metric map (its type and parameters).
    """
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
    """
    A set of map definitions, used to build a CMultiMetricMap.
    """
    def __getitem__(self, arg0: int) -> TMetricMapInitializer:
        ...
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def __iter__(self) -> typing.Iterator:
        ...
    def __len__(self) -> int:
        ...
    def __repr__(self) -> str:
        ...
    def clear(self) -> None:
        """
        Removes all map definitions.
        """
    def push_back(self, mapDefinition: TMetricMapInitializer) -> None:
        """
        Appends a map definition.
        """
    def size(self) -> int:
        """
        Returns the number of map definitions.
        """
class CSimpleMap(mrpt.serialization.CSerializable):
    """
    A view-based map: a set of poses and what the robot saw from those poses.
    """
    class Keyframe:
        """
        One keyframe of a CSimpleMap: a pose PDF, a sensory frame and an optional twist.
        """
        localTwist: mrpt.math.TTwist3D | None
        pose: mrpt.poses.CPose3DPDF
        sf: CSensoryFrame
        @typing.overload
        def __init__(self) -> None:
            """
            Default constructor.
            """
        @typing.overload
        def __init__(self, pose: mrpt.poses.CPose3DPDF, sf: CSensoryFrame, localTwist: mrpt.math.TTwist3D | None = None) -> None:
            """
            Builds a keyframe from a pose PDF, a sensory frame and an optional twist.
            """
    def __getitem__(self, arg0: int) -> CSimpleMap.Keyframe:
        ...
    def __init__(self) -> None:
        """
        Default ctor: empty map.
        """
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
        """
        Remove all stored keyframes.
        """
    def empty(self) -> bool:
        """
        Returns true if the map has no keyframes.
        """
    def get(self, index: int) -> CSimpleMap.Keyframe:
        """
        Returns the i-th keyframe.
        """
    @typing.overload
    def insert(self, pose: mrpt.poses.CPose3DPDF, sf: CSensoryFrame, localTwist: mrpt.math.TTwist3D | None = None) -> None:
        """
        Appends a keyframe
        """
    @typing.overload
    def insert(self, keyframe: CSimpleMap.Keyframe) -> None:
        """
        Appends a keyframe (pose PDF and sensory frame).
        """
    def loadFromFile(self, fileName: str) -> bool:
        """
        Loads a .simplemap file (possibly compressed). Returns False on error.
        """
    def remove(self, index: int) -> None:
        """
        Removes the i-th keyframe.
        """
    def saveToFile(self, fileName: str) -> bool:
        """
        Saves to a .simplemap file. Returns False on error.
        """
    def size(self) -> int:
        """
        Returns the number of keyframes in the map.
        """
class CRawlog(mrpt.serialization.CSerializable):
    """
    The main class for loading and processing robotics datasets, or "rawlogs".
    """
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
        """
        Default constructor.
        """
    def __iter__(self) -> typing.Iterator:
        """
        Iterates over all entries
        """
    def __len__(self) -> int:
        ...
    def __repr__(self) -> str:
        ...
    def clear(self) -> None:
        """
        Removes all entries.
        """
    def empty(self) -> bool:
        """
        Returns true if the rawlog is empty.
        """
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
        """
        Returns the type of a given element.
        """
    def insert(self, obj: mrpt.serialization.CSerializable) -> None:
        """
        Appends an object (CSensoryFrame, CActionCollection, CObservation, ...). The object is stored by reference, not copied.
        """
    def loadFromRawLogFile(self, fileName: str, non_obs_objects_are_legal: bool = False) -> bool:
        """
        Loads a .rawlog file (possibly compressed). Returns False on error.
        """
    def remove(self, index: int) -> None:
        """
        Removes the entry at the given index.
        """
    def saveToRawLogFile(self, fileName: str) -> bool:
        """
        Saves to a .rawlog file. Returns False on error.
        """
    def setCommentText(self, text: str) -> None:
        """
        Changes the block of comment text for the rawlog.
        """
    def size(self) -> int:
        """
        Returns the number of actions / observations object in the sequence.
        """
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
