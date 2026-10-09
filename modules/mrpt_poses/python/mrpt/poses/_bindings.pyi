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
class CPose3D(mrpt.serialization.CSerializable):
    """
    SE(3) rigid-body pose (x, y, z, yaw, pitch, roll), with a cached rotation matrix.
    """
    pitch: float
    roll: float
    yaw: float
    @staticmethod
    def FromTranslation(arg0: float, arg1: float, arg2: float) -> CPose3D:
        """
        Builds a pose with a translation without rotation.
        """
    @staticmethod
    def FromXYZYawPitchRoll(arg0: float, arg1: float, arg2: float, arg3: float, arg4: float, arg5: float) -> CPose3D:
        """
        Builds a pose from a translation (x,y,z) in meters and (yaw,pitch,roll) angles in radians.
        """
    @staticmethod
    def FromYawPitchRoll(arg0: float, arg1: float, arg2: float) -> CPose3D:
        """
        Builds a pose with a null translation and (yaw,pitch,roll) angles in radians.
        """
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
        """
        Default constructor, with all the coordinates set to zero.
        """
    @typing.overload
    def __init__(self, x: float, y: float, z: float, yaw: float = 0, pitch: float = 0, roll: float = 0) -> None:
        """
        Constructor with Initialization of the pose, translation (x,y,z) in meters, (yaw,pitch,roll) angles in radians.
        """
    @typing.overload
    def __init__(self, arg0: CPose2D) -> None:
        """
        Builds a 3D pose from a 2D pose (z, pitch and roll set to zero).
        """
    def __repr__(self) -> str:
        ...
    def __str__(self) -> str:
        ...
    def __sub__(self, arg0: CPose3D) -> CPose3D:
        ...
    def asString(self) -> str:
        """
        Returns a human-readable textual representation of the object (eg: "[x y z yaw pitch roll]", angles in degrees.)
        """
    def asTPose(self) -> mrpt.math.TPose3D:
        """
        Convert to lightweight mrpt.math.TPose3D
        """
    @typing.overload
    def composePoint(self, arg0: mrpt.math.TPoint3D) -> mrpt.math.TPoint3D:
        """
        Transforms a point from the local frame of this pose into the global frame.
        """
    @typing.overload
    def composePoint(self, arg0: float, arg1: float, arg2: float) -> mrpt.math.TPoint3D:
        """
        Transforms a point (x, y, z) from the local frame of this pose into the global frame.
        """
    def getHomogeneousMatrix(self) -> mrpt.math.CMatrixDouble44:
        """
        Returns the 4x4 homogeneous transformation matrix
        """
    def getInverseHomogeneousMatrix(self) -> mrpt.math.CMatrixDouble44:
        """
        Returns the corresponding 4x4 inverse homogeneous transformation matrix for this point or pose.
        """
    def getOppositeScalar(self) -> CPose3D:
        """
        Return the opposite of the current pose instance by taking the negative of all its components individually.
        """
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
        """
        Transforms a point from the global frame into the local frame of this pose.
        """
    @typing.overload
    def inverseComposePoint(self, arg0: float, arg1: float, arg2: float) -> mrpt.math.TPoint3D:
        """
        Transforms a point (x, y, z) from the global frame into the local frame of this pose.
        """
    def setFromValues(self, arg0: float, arg1: float, arg2: float, arg3: float, arg4: float, arg5: float) -> None:
        """
        Sets the pose from a position (meters) and yaw, pitch, roll angles (radians).
        """
    def setRotationMatrix(self, arg0: mrpt.math.CMatrixDouble33) -> None:
        """
        Sets the 3x3 rotation matrix.
        """
    def setYawPitchRoll(self, arg0: float, arg1: float, arg2: float) -> None:
        """
        Sets the three rotation angles, in radians.
        """
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
class CPose2D(mrpt.serialization.CSerializable):
    """
    SE(2) rigid-body pose (x, y, phi).
    """
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
    def __init__(self, p: CPose3D) -> None:
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
class CPosePDF(mrpt.serialization.CSerializable):
    """
    Base class of probability density functions (PDFs) of a 2D pose (x, y, phi).
    """
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
    """
    Base class of probability density functions (PDFs) of a 3D pose.
    """
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
    """
    A PDF of a 2D pose, as a set of weighted samples (particles).
    """
    def __init__(self, M: int = 1) -> None:
        """
        Creates M particles at the origin
        """
    def __len__(self) -> int:
        ...
    def __repr__(self) -> str:
        ...
    def clear(self) -> None:
        """
        Removes all particles.
        """
    def drawSingleSample(self) -> CPose2D:
        """
        Draws one sample from the distribution (weights must be normalized).
        """
    def getMostLikelyParticle(self) -> mrpt.math.TPose2D:
        """
        Returns the particle with the highest weight.
        """
    def getParticlePose(self, i: int) -> mrpt.math.TPose2D:
        """
        Returns the pose of the i'th particle.
        """
    def getParticlesAsNumpy(self) -> numpy.ndarray:
        """
        Returns all particles as an Nx4 array with columns (x, y, phi, log_weight)
        """
    def resetAroundSetOfPoses(self, list_poses: list[mrpt.math.TPose2D], num_particles_per_pose: int, spread_x: float, spread_y: float, spread_phi_rad: float) -> None:
        """
        Resets the particles around a set of poses (x, y, phi), with a given number of particles per pose and spread.
        """
    def resetDeterministic(self, location: mrpt.math.TPose2D, particlesCount: int = 0) -> None:
        """
        Sets all particles to the given pose (particlesCount=0 keeps the count)
        """
    def resetUniform(self, x_min: float, x_max: float, y_min: float, y_max: float, phi_min: float = -3.141592653589793, phi_max: float = 3.141592653589793, particlesCount: int = -1) -> None:
        """
        Spreads particles uniformly in [x_min,x_max]x[y_min,y_max]x[phi_min,phi_max]
        """
    def size(self) -> int:
        """
        Returns the number of particles.
        """
class CPose3DPDFParticles(CPose3DPDF, mrpt.bayes.CParticleFilterCapable):
    """
    A PDF of a 3D pose, as a set of weighted samples (particles).
    """
    def __init__(self, M: int = 1) -> None:
        """
        Creates M particles at the origin
        """
    def __len__(self) -> int:
        ...
    def __repr__(self) -> str:
        ...
    def drawSingleSample(self) -> CPose3D:
        """
        Draws one sample from the distribution (weights must be normalized).
        """
    def getMostLikelyParticle(self) -> mrpt.math.TPose3D:
        """
        Returns the particle with the highest weight.
        """
    def getParticlePose(self, i: int) -> mrpt.math.TPose3D:
        """
        Returns the pose of the i'th particle.
        """
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
        """
        Returns the number of particles.
        """
class CPose3DPDFGaussian(CPose3DPDF):
    """
    A PDF of a 3D pose as a Gaussian with a mean and a 6x6 covariance matrix.
    """
    cov: mrpt.math.CMatrixDouble66
    mean: CPose3D
    def __add__(self, arg0: CPose3DPDFGaussian) -> CPose3DPDFGaussian:
        ...
    def __iadd__(self, arg0: CPose3D) -> None:
        ...
    @typing.overload
    def __init__(self) -> None:
        """
        Default constructor.
        """
    @typing.overload
    def __init__(self, arg0: CPose3D) -> None:
        """
        Builds the PDF from a mean, with zero covariance.
        """
    @typing.overload
    def __init__(self, arg0: CPose3D, arg1: mrpt.math.CMatrixDouble66) -> None:
        """
        Builds the PDF from a mean and a 6x6 covariance matrix.
        """
    def __str__(self) -> str:
        ...
    def drawSingleSample(self) -> CPose3D:
        """
        Draws a single sample from the Gaussian distribution and returns it as a CPose3D.
        """
    def evaluateNormalizedPDF(self, arg0: CPose3D) -> float:
        """
        Evaluates the ratio PDF(x) / PDF(MEAN), that is, the normalized PDF in the range [0,1].
        """
    def evaluatePDF(self, arg0: CPose3D) -> float:
        """
        Evaluates the PDF at a given point.
        """
    def saveToTextFile(self, arg0: str) -> bool:
        """
        Saves the mean and covariance to a text file.
        """
class CPose3DPDFGaussianInf(CPose3DPDF):
    """
    A PDF of a 3D pose as a Gaussian with a mean and a 6x6 information (inverse covariance) matrix.
    """
    cov_inv: mrpt.math.CMatrixDouble66
    mean: CPose3D
    @typing.overload
    def __init__(self) -> None:
        """
        Default constructor: zero mean and zero information matrix.
        """
    @typing.overload
    def __init__(self, arg0: CPose3D) -> None:
        """
        Builds the PDF from a mean, with zero information matrix.
        """
    @typing.overload
    def __init__(self, mean: CPose3D, inf_matrix: mrpt.math.CMatrixDouble66) -> None:
        """
        Builds the PDF from a mean and a 6x6 information matrix.
        """
    def drawSingleSample(self) -> CPose3D:
        """
        Draws a single sample from the distribution and returns it as a CPose3D.
        """
    def isInfType(self) -> bool:
        """
        Returns whether the class instance holds the uncertainty in covariance or information form.
        """
class SE_average2:
    """
    Computes the (optionally weighted) average of a set of SE(2) poses.
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    @typing.overload
    def append(self, arg0: CPose2D) -> None:
        """
        Adds a pose with unit weight.
        """
    @typing.overload
    def append(self, arg0: CPose2D, arg1: float) -> None:
        """
        Adds a pose with the given weight.
        """
    def clear(self) -> None:
        """
        Resets the accumulated poses.
        """
    def get_average(self) -> CPose2D:
        """
        Returns the calculated average pose.
        """
class SE_average3:
    """
    Computes the (optionally weighted) average of a set of SE(3) poses.
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    @typing.overload
    def append(self, arg0: CPose3D) -> None:
        """
        Adds a pose with unit weight.
        """
    @typing.overload
    def append(self, arg0: CPose3D, arg1: float) -> None:
        """
        Adds a pose with the given weight.
        """
    def clear(self) -> None:
        """
        Resets the accumulated poses.
        """
    def get_average(self) -> CPose3D:
        """
        Returns the calculated average pose.
        """
class CPoint2D(mrpt.serialization.CSerializable):
    """
    A class used to store a 2D point.
    """
    x: float
    y: float
    @staticmethod
    def fromTPoint(arg0: mrpt.math.TPoint2D) -> CPoint2D:
        """
        Construct CPoint2D from a lightweight mrpt.math.TPoint2D
        """
    @typing.overload
    def __init__(self) -> None:
        """
        Default constructor.
        """
    @typing.overload
    def __init__(self, x: float, y: float) -> None:
        """
        Constructor for initializing point coordinates.
        """
    def __repr__(self) -> str:
        ...
    def __str__(self) -> str:
        ...
    def asString(self) -> str:
        """
        Returns a text representation, e.g. "[0.02 1.04]".
        """
    def asTPoint(self) -> mrpt.math.TPoint2D:
        """
        Convert to lightweight mrpt.math.TPoint2D
        """
class CPoint3D(mrpt.serialization.CSerializable):
    """
    A class used to store a 3D point.
    """
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
        """
        Default constructor.
        """
    @typing.overload
    def __init__(self, x: float, y: float, z: float) -> None:
        """
        Constructor for initializing point coordinates.
        """
    def __repr__(self) -> str:
        ...
    def __str__(self) -> str:
        ...
    def asString(self) -> str:
        """
        Returns a text representation, e.g. "[0.02 1.04 -0.80]".
        """
    def asTPoint(self) -> mrpt.math.TPoint3D:
        """
        Convert to lightweight mrpt.math.TPoint3D
        """
class CPose3DQuat(mrpt.serialization.CSerializable):
    """
    A class used to store a 3D pose as a translation (x,y,z) and a quaternion (qr,qx,qy,qz).
    """
    quat: mrpt.math.CQuaternionDouble
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
        """
        Default constructor, initialize translation to zeros and quaternion to no rotation.
        """
    @typing.overload
    def __init__(self, arg0: CPose3D) -> None:
        """
        Builds the pose from a CPose3D.
        """
    def __repr__(self) -> str:
        ...
    def __str__(self) -> str:
        ...
    def asString(self) -> str:
        """
        Returns a human-readable textual representation of the object as: "[x y z qw qx qy qz]".
        """
    def asTPose(self) -> mrpt.math.TPose3DQuat:
        """
        Convert to lightweight mrpt.math.TPose3DQuat
        """
    def norm(self) -> float:
        """
        Returns the Euclidean norm of the translation part.
        """
class CPosePDFGaussian(CPosePDF):
    """
    A PDF of a 2D pose as a Gaussian with a mean and a 3x3 covariance matrix.
    """
    cov: mrpt.math.CMatrixDouble33
    mean: CPose2D
    @typing.overload
    def __init__(self) -> None:
        """
        Default constructor.
        """
    @typing.overload
    def __init__(self, arg0: CPose2D) -> None:
        """
        Builds the PDF from a mean, with zero covariance.
        """
    @typing.overload
    def __init__(self, arg0: CPose2D, arg1: mrpt.math.CMatrixDouble33) -> None:
        """
        Builds the PDF from a mean and a 3x3 covariance matrix.
        """
    def __repr__(self) -> str:
        ...
    def __str__(self) -> str:
        ...
    def drawSingleSample(self) -> CPose2D:
        """
        Draw a single sample from the Gaussian distribution
        """
    def evaluateNormalizedPDF(self, arg0: CPose2D) -> float:
        """
        Evaluates the ratio PDF(x) / PDF(MEAN), that is, the normalized PDF in the range [0,1].
        """
    def evaluatePDF(self, arg0: CPose2D) -> float:
        """
        Evaluates the PDF at a given point.
        """
class CPosePDFGaussianInf(CPosePDF):
    """
    A PDF of a 2D pose as a Gaussian with a mean and a 3x3 information (inverse covariance) matrix.
    """
    cov_inv: mrpt.math.CMatrixDouble33
    mean: CPose2D
    @typing.overload
    def __init__(self) -> None:
        """
        Default constructor: zero mean and zero information matrix.
        """
    @typing.overload
    def __init__(self, arg0: CPose2D) -> None:
        """
        Builds the PDF from a mean, with zero information matrix.
        """
    @typing.overload
    def __init__(self, mean: CPose2D, inf_matrix: mrpt.math.CMatrixDouble33) -> None:
        """
        Builds the PDF from a mean and a 3x3 information matrix.
        """
    def __repr__(self) -> str:
        ...
    def __str__(self) -> str:
        ...
    def drawSingleSample(self) -> CPose2D:
        """
        Draws a single sample from the distribution.
        """
class CPose2DInterpolator(mrpt.serialization.CSerializable):
    """
    A time-stamped trajectory in SE(2), with interpolation between poses.
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def __len__(self) -> int:
        ...
    def clear(self) -> None:
        """
        Clears the current sequence of poses.
        """
    def empty(self) -> bool:
        """
        Returns true if the trajectory has no poses.
        """
    def insert(self, arg0: datetime.timedelta, arg1: mrpt.math.TPose2D) -> None:
        """
        Inserts a new pose in the sequence. It overwrites any previously existing pose at exactly the same time.
        """
    def interpolate(self, arg0: datetime.timedelta) -> tuple:
        """
        Returns (TPose2D, valid) — interpolated pose at given time
        """
    def size(self) -> int:
        """
        Returns the number of poses in the trajectory.
        """
class CPose3DInterpolator(mrpt.serialization.CSerializable):
    """
    A time-stamped trajectory in SE(3), with interpolation between poses.
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def __len__(self) -> int:
        ...
    def clear(self) -> None:
        """
        Clears the current sequence of poses.
        """
    def empty(self) -> bool:
        """
        Returns true if the trajectory has no poses.
        """
    def insert(self, arg0: datetime.timedelta, arg1: mrpt.math.TPose3D) -> None:
        """
        Inserts a new pose in the sequence. It overwrites any previously existing pose at exactly the same time.
        """
    def interpolate(self, arg0: datetime.timedelta) -> tuple:
        """
        Returns (TPose3D, valid) — interpolated pose at given time
        """
    def size(self) -> int:
        """
        Returns the number of poses in the trajectory.
        """
class CPoseRandomSampler:
    """
    An efficient generator of random samples drawn from a given 2D (CPosePDF) or 3D (CPose3DPDF) pose probability density function (pdf).
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
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
        """
        Sets the 2D pose PDF to draw samples from.
        """
    @typing.overload
    def setPosePDF(self, arg0: CPose3DPDF) -> None:
        """
        Sets the 3D pose PDF to draw samples from.
        """
