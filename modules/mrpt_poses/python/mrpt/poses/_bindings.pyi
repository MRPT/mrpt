"""
Python bindings for mrpt_poses
"""
from __future__ import annotations
import datetime
import mrpt.bayes
import mrpt.math
import mrpt.serialization
import numpy
import typing
__all__: list[str] = ['CPoint2D', 'CPoint3D', 'CPose2D', 'CPose2DInterpolator', 'CPose3D', 'CPose3DInterpolator', 'CPose3DPDF', 'CPose3DPDFGaussian', 'CPose3DPDFGaussianInf', 'CPose3DPDFParticles', 'CPose3DQuat', 'CPosePDF', 'CPosePDFGaussian', 'CPosePDFGaussianInf', 'CPosePDFParticles', 'CPoseRandomSampler', 'SE_average2', 'SE_average3']
class CPose2D(mrpt.serialization.CSerializable):
    @staticmethod
    def fromTPose(arg0: mrpt.math.TPose2D) -> CPose2D:
        """
        Construct CPose2D from a lightweight mrpt.math.TPose2D
        """
    def __add__(self, arg0: CPose2D) -> CPose2D:
        ...
    def __iadd__(self, arg0: CPose2D) -> CPose2D:
        ...
    @typing.overload
    def __init__(self) -> None:
        """
        Default constructor (0,0,0).
        """
    @typing.overload
    def __init__(self, x: float, y: float, phi: float) -> None:
        """
        Constructor from coordinates.
        """
    @typing.overload
    def __init__(self, arg0: typing.Any) -> None:  # unnamed C++ type
        """
        Construct from CPose3D (loss of z/pitch/roll).
        """
    def __repr__(self) -> str:
        ...
    def __str__(self) -> str:
        ...
    def __sub__(self, arg0: CPose2D) -> CPose2D:
        ...
    def asString(self) -> str:
        """
        Returns human-readable string [x y phi]
        """
    def asTPose(self) -> mrpt.math.TPose2D:
        """
        Convert to lightweight mrpt.math.TPose2D
        """
    def fromString(self, arg0: str) -> None:
        """
        Set value from string
        """
    def inverse(self) -> CPose2D:
        """
        Returns the inverse pose
        """
    def norm(self) -> float:
        """
        Returns the norm of the (x,y) vector
        """
    def normalizePhi(self) -> None:
        """
        Forces phi to be in [-pi,pi]
        """
    @property
    def phi(self) -> float:
        """
        Phi orientation (radians)
        """
    @phi.setter
    def phi(self, arg1: float) -> None:
        ...
    @property
    def x(self) -> float:
        """
        X coordinate
        """
    @x.setter
    def x(self, arg1: float) -> None:
        ...
    @property
    def y(self) -> float:
        """
        Y coordinate
        """
    @y.setter
    def y(self, arg1: float) -> None:
        ...
class CPose3D(mrpt.serialization.CSerializable):
    pitch: float
    roll: float
    yaw: float
    @staticmethod
    def FromTranslation(arg0: float, arg1: float, arg2: float) -> CPose3D:
        ...
    @staticmethod
    def FromXYZYawPitchRoll(arg0: float, arg1: float, arg2: float, arg3: float, arg4: float, arg5: float) -> CPose3D:
        ...
    @staticmethod
    def FromYawPitchRoll(arg0: float, arg1: float, arg2: float) -> CPose3D:
        ...
    @staticmethod
    def fromTPose(arg0: mrpt.math.TPose3D) -> CPose3D:
        """
        Construct CPose3D from a lightweight mrpt.math.TPose3D
        """
    def __add__(self, arg0: CPose3D) -> CPose3D:
        ...
    def __iadd__(self, arg0: CPose3D) -> CPose3D:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, x: float, y: float, z: float, yaw: float = 0, pitch: float = 0, roll: float = 0) -> None:
        ...
    @typing.overload
    def __init__(self, arg0: CPose2D) -> None:
        ...
    def __repr__(self) -> str:
        ...
    def __str__(self) -> str:
        ...
    def __sub__(self, arg0: CPose3D) -> CPose3D:
        ...
    def asString(self) -> str:
        ...
    def asTPose(self) -> mrpt.math.TPose3D:
        """
        Convert to lightweight mrpt.math.TPose3D
        """
    @typing.overload
    def composePoint(self, arg0: mrpt.math.TPoint3D) -> mrpt.math.TPoint3D:
        ...
    @typing.overload
    def composePoint(self, arg0: float, arg1: float, arg2: float) -> mrpt.math.TPoint3D:
        ...
    def getHomogeneousMatrix(self) -> mrpt.math.CMatrixDouble44:
        """
        Returns the 4x4 homogeneous transformation matrix
        """
    def getInverseHomogeneousMatrix(self) -> mrpt.math.CMatrixDouble44:
        ...
    def getOppositeScalar(self) -> CPose3D:
        ...
    def getRotationMatrix(self) -> mrpt.math.CMatrixDouble33:
        """
        Returns the 3x3 Rotation Matrix
        """
    def getYawPitchRoll(self) -> tuple[float, float, float]:
        """
        Returns (yaw, pitch, roll) as a tuple in radians
        """
    def inverse(self) -> None:
        """
        Inverts the pose in place
        """
    @typing.overload
    def inverseComposePoint(self, arg0: mrpt.math.TPoint3D) -> mrpt.math.TPoint3D:
        ...
    @typing.overload
    def inverseComposePoint(self, arg0: float, arg1: float, arg2: float) -> mrpt.math.TPoint3D:
        ...
    def setFromValues(self, arg0: float, arg1: float, arg2: float, arg3: float, arg4: float, arg5: float) -> None:
        ...
    def setRotationMatrix(self, arg0: mrpt.math.CMatrixDouble33) -> None:
        ...
    def setYawPitchRoll(self, arg0: float, arg1: float, arg2: float) -> None:
        ...
    @property
    def x(self) -> float:
        """
        X coordinate
        """
    @x.setter
    def x(self, arg1: float) -> None:
        ...
    @property
    def y(self) -> float:
        """
        Y coordinate
        """
    @y.setter
    def y(self, arg1: float) -> None:
        ...
    @property
    def z(self) -> float:
        """
        Z coordinate
        """
    @z.setter
    def z(self, arg1: float) -> None:
        ...
class CPosePDF(mrpt.serialization.CSerializable):
    def __str__(self) -> str:
        ...
    def getCovariance(self) -> mrpt.math.CMatrixDouble33:
        """
        Returns the 3x3 covariance matrix
        """
    def getCovarianceAndMean(self) -> tuple:
        """
        Returns the tuple (cov: CMatrixDouble33, mean: CPose2D)
        """
    def getMean(self) -> CPose2D:
        """
        Returns the mean (expected value) of the distribution
        """
    def saveToTextFile(self, file: str) -> bool:
        """
        Saves the distribution to a text file. Returns False on error.
        """
class CPose3DPDF(mrpt.serialization.CSerializable):
    def __str__(self) -> str:
        ...
    def getCovariance(self) -> mrpt.math.CMatrixDouble66:
        """
        Returns the 6x6 covariance matrix
        """
    def getCovarianceAndMean(self) -> tuple:
        """
        Returns the tuple (cov: CMatrixDouble66, mean: CPose3D)
        """
    def getMean(self) -> CPose3D:
        """
        Returns the mean (expected value) of the distribution
        """
    def saveToTextFile(self, file: str) -> bool:
        """
        Saves the distribution to a text file. Returns False on error.
        """
class CPosePDFParticles(CPosePDF, mrpt.bayes.CParticleFilterCapable):
    def __init__(self, M: int = 1) -> None:
        """
        Creates M particles at the origin
        """
    def __len__(self) -> int:
        ...
    def __repr__(self) -> str:
        ...
    def clear(self) -> None:
        ...
    def drawSingleSample(self) -> CPose2D:
        ...
    def getMostLikelyParticle(self) -> mrpt.math.TPose2D:
        ...
    def getParticlePose(self, i: int) -> mrpt.math.TPose2D:
        ...
    def getParticlesAsNumpy(self) -> numpy.ndarray:
        """
        Returns all particles as an Nx4 array with columns (x, y, phi, log_weight)
        """
    def resetAroundSetOfPoses(self, list_poses: list[mrpt.math.TPose2D], num_particles_per_pose: int, spread_x: float, spread_y: float, spread_phi_rad: float) -> None:
        ...
    def resetDeterministic(self, location: mrpt.math.TPose2D, particlesCount: int = 0) -> None:
        """
        Sets all particles to the given pose (particlesCount=0 keeps the count)
        """
    def resetUniform(self, x_min: float, x_max: float, y_min: float, y_max: float, phi_min: float = -3.141592653589793, phi_max: float = 3.141592653589793, particlesCount: int = -1) -> None:
        """
        Spreads particles uniformly in [x_min,x_max]x[y_min,y_max]x[phi_min,phi_max]
        """
    def size(self) -> int:
        ...
class CPose3DPDFParticles(CPose3DPDF, mrpt.bayes.CParticleFilterCapable):
    def __init__(self, M: int = 1) -> None:
        """
        Creates M particles at the origin
        """
    def __len__(self) -> int:
        ...
    def __repr__(self) -> str:
        ...
    def drawSingleSample(self) -> CPose3D:
        ...
    def getMostLikelyParticle(self) -> mrpt.math.TPose3D:
        ...
    def getParticlePose(self, i: int) -> mrpt.math.TPose3D:
        ...
    def getParticlesAsNumpy(self) -> numpy.ndarray:
        """
        Returns all particles as an Nx7 array with columns (x, y, z, yaw, pitch, roll, log_weight)
        """
    def resetDeterministic(self, location: mrpt.math.TPose3D, particlesCount: int = 0) -> None:
        """
        Sets all particles to the given pose (particlesCount=0 keeps the count)
        """
    def resetUniform(self, corner_min: mrpt.math.TPose3D, corner_max: mrpt.math.TPose3D, particlesCount: int = -1) -> None:
        """
        Spreads particles uniformly between two TPose3D corners
        """
    def size(self) -> int:
        ...
class CPose3DPDFGaussian(CPose3DPDF):
    cov: mrpt.math.CMatrixDouble66
    mean: CPose3D
    def __add__(self, arg0: CPose3DPDFGaussian) -> CPose3DPDFGaussian:
        ...
    def __iadd__(self, arg0: CPose3D) -> None:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, arg0: CPose3D) -> None:
        ...
    @typing.overload
    def __init__(self, arg0: CPose3D, arg1: mrpt.math.CMatrixDouble66) -> None:
        ...
    def __str__(self) -> str:
        ...
    def drawSingleSample(self) -> CPose3D:
        """
        Draws a single sample from the Gaussian distribution and returns it as a CPose3D.
        """
    def evaluateNormalizedPDF(self, arg0: CPose3D) -> float:
        ...
    def evaluatePDF(self, arg0: CPose3D) -> float:
        ...
    def saveToTextFile(self, arg0: str) -> bool:
        ...
class CPose3DPDFGaussianInf(CPose3DPDF):
    cov_inv: mrpt.math.CMatrixDouble66
    mean: CPose3D
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, arg0: CPose3D) -> None:
        ...
    @typing.overload
    def __init__(self, mean: CPose3D, inf_matrix: mrpt.math.CMatrixDouble66) -> None:
        ...
    def drawSingleSample(self) -> CPose3D:
        """
        Draws a single sample from the distribution and returns it as a CPose3D.
        """
    def isInfType(self) -> bool:
        ...
class SE_average2:
    def __init__(self) -> None:
        ...
    @typing.overload
    def append(self, arg0: CPose2D) -> None:
        ...
    @typing.overload
    def append(self, arg0: CPose2D, arg1: float) -> None:
        ...
    def clear(self) -> None:
        ...
    def get_average(self) -> CPose2D:
        """
        Returns the calculated average pose.
        """
class SE_average3:
    def __init__(self) -> None:
        ...
    @typing.overload
    def append(self, arg0: CPose3D) -> None:
        ...
    @typing.overload
    def append(self, arg0: CPose3D, arg1: float) -> None:
        ...
    def clear(self) -> None:
        ...
    def get_average(self) -> CPose3D:
        """
        Returns the calculated average pose.
        """
class CPoint2D(mrpt.serialization.CSerializable):
    x: float
    y: float
    @staticmethod
    def fromTPoint(arg0: mrpt.math.TPoint2D) -> CPoint2D:
        """
        Construct CPoint2D from a lightweight mrpt.math.TPoint2D
        """
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, x: float, y: float) -> None:
        ...
    def __repr__(self) -> str:
        ...
    def __str__(self) -> str:
        ...
    def asString(self) -> str:
        ...
    def asTPoint(self) -> mrpt.math.TPoint2D:
        """
        Convert to lightweight mrpt.math.TPoint2D
        """
class CPoint3D(mrpt.serialization.CSerializable):
    x: float
    y: float
    z: float
    @staticmethod
    def fromTPoint(arg0: mrpt.math.TPoint3D) -> CPoint3D:
        """
        Construct CPoint3D from a lightweight mrpt.math.TPoint3D
        """
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, x: float, y: float, z: float) -> None:
        ...
    def __repr__(self) -> str:
        ...
    def __str__(self) -> str:
        ...
    def asString(self) -> str:
        ...
    def asTPoint(self) -> mrpt.math.TPoint3D:
        """
        Convert to lightweight mrpt.math.TPoint3D
        """
class CPose3DQuat(mrpt.serialization.CSerializable):
    quat: typing.Any  # unnamed C++ type
    x: float
    y: float
    z: float
    @staticmethod
    def fromTPose(arg0: mrpt.math.TPose3DQuat) -> CPose3DQuat:
        """
        Construct CPose3DQuat from a lightweight mrpt.math.TPose3DQuat
        """
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, arg0: CPose3D) -> None:
        ...
    def __repr__(self) -> str:
        ...
    def __str__(self) -> str:
        ...
    def asString(self) -> str:
        ...
    def asTPose(self) -> mrpt.math.TPose3DQuat:
        """
        Convert to lightweight mrpt.math.TPose3DQuat
        """
    def norm(self) -> float:
        ...
class CPosePDFGaussian(CPosePDF):
    cov: mrpt.math.CMatrixDouble33
    mean: CPose2D
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, arg0: CPose2D) -> None:
        ...
    @typing.overload
    def __init__(self, arg0: CPose2D, arg1: mrpt.math.CMatrixDouble33) -> None:
        ...
    def __repr__(self) -> str:
        ...
    def __str__(self) -> str:
        ...
    def drawSingleSample(self) -> CPose2D:
        """
        Draw a single sample from the Gaussian distribution
        """
    def evaluateNormalizedPDF(self, arg0: CPose2D) -> float:
        ...
    def evaluatePDF(self, arg0: CPose2D) -> float:
        ...
class CPosePDFGaussianInf(CPosePDF):
    cov_inv: mrpt.math.CMatrixDouble33
    mean: CPose2D
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, arg0: CPose2D) -> None:
        ...
    @typing.overload
    def __init__(self, mean: CPose2D, inf_matrix: mrpt.math.CMatrixDouble33) -> None:
        ...
    def __repr__(self) -> str:
        ...
    def __str__(self) -> str:
        ...
    def drawSingleSample(self) -> CPose2D:
        ...
class CPose2DInterpolator(mrpt.serialization.CSerializable):
    def __init__(self) -> None:
        ...
    def __len__(self) -> int:
        ...
    def clear(self) -> None:
        ...
    def empty(self) -> bool:
        ...
    def insert(self, arg0: datetime.timedelta, arg1: mrpt.math.TPose2D) -> None:
        ...
    def interpolate(self, arg0: datetime.timedelta) -> tuple:
        """
        Returns (TPose2D, valid) — interpolated pose at given time
        """
    def size(self) -> int:
        ...
class CPose3DInterpolator(mrpt.serialization.CSerializable):
    def __init__(self) -> None:
        ...
    def __len__(self) -> int:
        ...
    def clear(self) -> None:
        ...
    def empty(self) -> bool:
        ...
    def insert(self, arg0: datetime.timedelta, arg1: mrpt.math.TPose3D) -> None:
        ...
    def interpolate(self, arg0: datetime.timedelta) -> tuple:
        """
        Returns (TPose3D, valid) — interpolated pose at given time
        """
    def size(self) -> int:
        ...
class CPoseRandomSampler:
    def __init__(self) -> None:
        ...
    def drawSample2D(self) -> CPose2D:
        """
        Draw a 2D pose sample
        """
    def drawSample3D(self) -> CPose3D:
        """
        Draw a 3D pose sample
        """
    @typing.overload
    def setPosePDF(self, arg0: CPosePDF) -> None:
        ...
    @typing.overload
    def setPosePDF(self, arg0: CPose3DPDF) -> None:
        ...
