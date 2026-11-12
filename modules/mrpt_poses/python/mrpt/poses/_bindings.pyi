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
__all__: list[str] = ['CPoint2D', 'CPoint3D', 'CPose2D', 'CPose2DInterpolator', 'CPose3D', 'CPose3DInterpolator', 'CPose3DPDF', 'CPose3DPDFGaussian', 'CPose3DPDFGaussianInf', 'CPose3DPDFParticles', 'CPose3DQuat', 'CPosePDF', 'CPosePDFGaussian', 'CPosePDFGaussianInf', 'CPosePDFParticles', 'CPosePDFSOG', 'CPoseRandomSampler', 'SE_average2', 'SE_average3', 'TInterpolatorMethod', 'imLinear2Neig', 'imLinear4Neig', 'imLinearSlerp', 'imSSLLLL', 'imSSLSLL', 'imSpline', 'imSplineSlerp']
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
    def __copy__(self) -> CPose3D:
        """
        Returns a copy of this object.
        """
    def __deepcopy__(self, memo: dict) -> CPose3D:
        """
        Returns a copy of this object.
        """
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
    @typing.overload
    def __init__(self, p: mrpt.math.TPose3D) -> None:
        """
        Builds the pose from a lightweight mrpt.math.TPose3D.
        """
    @typing.overload
    def __init__(self, other: CPose3D) -> None:
        """
        Copy constructor.
        """
    def __neg__(self) -> CPose3D:
        """
        Returns the inverse pose as a new object (SE(3) group inverse, not the negation of each coordinate). This pose is not modified.
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
        Inverts this pose in place and returns None. Use -p to get the inverse as a new pose instead (note that CPose2D.inverse() returns a new pose).
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
    def __copy__(self) -> CPose2D:
        """
        Returns a copy of this object.
        """
    def __deepcopy__(self, memo: dict) -> CPose2D:
        """
        Returns a copy of this object.
        """
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
    @typing.overload
    def __init__(self, p: mrpt.math.TPose2D) -> None:
        """
        Construct from a lightweight mrpt.math.TPose2D.
        """
    @typing.overload
    def __init__(self, other: CPose2D) -> None:
        """
        Copy constructor.
        """
    def __neg__(self) -> CPose2D:
        """
        Returns the inverse pose (SE(2) group inverse, not the negation of each coordinate).
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
    @typing.overload
    def composePoint(self, l: mrpt.math.TPoint2D) -> mrpt.math.TPoint2D:
        """
        Transforms a point from the local frame of this pose into the global frame.
        """
    @typing.overload
    def composePoint(self, lx: float, ly: float) -> mrpt.math.TPoint2D:
        """
        Transforms a point (x, y) from the local frame of this pose into the global frame.
        """
    @typing.overload
    def composePoint(self, l: mrpt.math.TPoint3D) -> mrpt.math.TPoint3D:
        """
        Transforms a 3D point from the local frame of this pose into the global frame (z is not modified).
        """
    def fromString(self, arg0: str) -> None:
        """
        Set value from string
        """
    def inverse(self) -> CPose2D:
        """
        Returns the inverse pose as a new object (same as -p), without modifying this one. Note that CPose3D.inverse() works in place instead.
        """
    @typing.overload
    def inverseComposePoint(self, g: mrpt.math.TPoint2D) -> mrpt.math.TPoint2D:
        """
        Transforms a point from the global frame into the local frame of this pose.
        """
    @typing.overload
    def inverseComposePoint(self, gx: float, gy: float) -> mrpt.math.TPoint2D:
        """
        Transforms a point (x, y) from the global frame into the local frame of this pose.
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
    def __copy__(self) -> CPosePDFParticles:
        """
        Returns a copy of this object.
        """
    def __deepcopy__(self, memo: dict) -> CPosePDFParticles:
        """
        Returns a copy of this object.
        """
    @typing.overload
    def __init__(self, M: int = 1) -> None:
        """
        Creates M particles at the origin
        """
    @typing.overload
    def __init__(self, other: CPosePDFParticles) -> None:
        """
        Copy constructor.
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
    def __copy__(self) -> CPose3DPDFParticles:
        """
        Returns a copy of this object.
        """
    def __deepcopy__(self, memo: dict) -> CPose3DPDFParticles:
        """
        Returns a copy of this object.
        """
    @typing.overload
    def __init__(self, M: int = 1) -> None:
        """
        Creates M particles at the origin
        """
    @typing.overload
    def __init__(self, other: CPose3DPDFParticles) -> None:
        """
        Copy constructor.
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
    def __copy__(self) -> CPose3DPDFGaussian:
        """
        Returns a copy of this object.
        """
    def __deepcopy__(self, memo: dict) -> CPose3DPDFGaussian:
        """
        Returns a copy of this object.
        """
    @typing.overload
    def __iadd__(self, arg0: CPose3D) -> CPose3DPDFGaussian:
        """
        Makes this = this (+) Ap, for a pose without uncertainty.
        """
    @typing.overload
    def __iadd__(self, arg0: CPose3DPDFGaussian) -> CPose3DPDFGaussian:
        """
        Makes this = this (+) Ap, propagating both covariances.
        """
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
    @typing.overload
    def __init__(self, other: CPose3DPDFGaussian) -> None:
        """
        Copy constructor.
        """
    def __isub__(self, arg0: CPose3DPDFGaussian) -> CPose3DPDFGaussian:
        """
        Makes this = this (-) Ap, propagating both covariances.
        """
    def __neg__(self) -> CPose3DPDFGaussian:
        """
        Returns the PDF of the inverse pose, (0,0,0) (-) this.
        """
    def __str__(self) -> str:
        ...
    def __sub__(self, arg0: CPose3DPDFGaussian) -> CPose3DPDFGaussian:
        """
        Inverse pose composition a (-) b, propagating both covariances (independent variables).
        """
    def changeCoordinatesReference(self, newReferenceBase: CPose3D) -> None:
        """
        Makes this = newReferenceBase (+) this, for a reference pose without uncertainty.
        """
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
    def inverse(self) -> CPose3DPDFGaussian:
        """
        Returns the PDF of the inverse pose, (0,0,0) (-) this, as a new object (same as -pdf). This PDF is not modified.
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
    def __copy__(self) -> CPose3DPDFGaussianInf:
        """
        Returns a copy of this object.
        """
    def __deepcopy__(self, memo: dict) -> CPose3DPDFGaussianInf:
        """
        Returns a copy of this object.
        """
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
    @typing.overload
    def __init__(self, other: CPose3DPDFGaussianInf) -> None:
        """
        Copy constructor.
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
    def __copy__(self) -> CPoint2D:
        """
        Returns a copy of this object.
        """
    def __deepcopy__(self, memo: dict) -> CPoint2D:
        """
        Returns a copy of this object.
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
    @typing.overload
    def __init__(self, other: CPoint2D) -> None:
        """
        Copy constructor.
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
    def __copy__(self) -> CPoint3D:
        """
        Returns a copy of this object.
        """
    def __deepcopy__(self, memo: dict) -> CPoint3D:
        """
        Returns a copy of this object.
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
    @typing.overload
    def __init__(self, other: CPoint3D) -> None:
        """
        Copy constructor.
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
    def __copy__(self) -> CPose3DQuat:
        """
        Returns a copy of this object.
        """
    def __deepcopy__(self, memo: dict) -> CPose3DQuat:
        """
        Returns a copy of this object.
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
    @typing.overload
    def __init__(self, p: mrpt.math.TPose3DQuat) -> None:
        """
        Builds the pose from a lightweight mrpt.math.TPose3DQuat.
        """
    @typing.overload
    def __init__(self, other: CPose3DQuat) -> None:
        """
        Copy constructor.
        """
    def __neg__(self) -> CPose3DQuat:
        """
        Returns the inverse pose as a new object (SE(3) group inverse, not the negation of each coordinate).
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
    def __add__(self, arg0: CPosePDFGaussian) -> CPosePDFGaussian:
        """
        Pose composition a (+) b, propagating both covariances (independent variables).
        """
    def __copy__(self) -> CPosePDFGaussian:
        """
        Returns a copy of this object.
        """
    def __deepcopy__(self, memo: dict) -> CPosePDFGaussian:
        """
        Returns a copy of this object.
        """
    @typing.overload
    def __iadd__(self, arg0: CPose2D) -> CPosePDFGaussian:
        """
        Makes this = this (+) Ap, for a pose without uncertainty.
        """
    @typing.overload
    def __iadd__(self, arg0: CPosePDFGaussian) -> CPosePDFGaussian:
        """
        Makes this = this (+) Ap, propagating both covariances.
        """
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
    @typing.overload
    def __init__(self, other: CPosePDFGaussian) -> None:
        """
        Copy constructor.
        """
    def __isub__(self, arg0: CPosePDFGaussian) -> CPosePDFGaussian:
        """
        Makes this = this (-) ref, propagating both covariances.
        """
    def __radd__(self, arg0: CPose2D) -> CPosePDFGaussian:
        """
        Composition p (+) pdf of a pose p without uncertainty and this PDF.
        """
    def __repr__(self) -> str:
        ...
    def __str__(self) -> str:
        ...
    def __sub__(self, arg0: CPosePDFGaussian) -> CPosePDFGaussian:
        """
        Inverse pose composition a (-) b, propagating both covariances (independent variables).
        """
    @typing.overload
    def changeCoordinatesReference(self, newReferenceBase: CPose2D) -> None:
        """
        Makes this = newReferenceBase (+) this, for a reference pose without uncertainty.
        """
    @typing.overload
    def changeCoordinatesReference(self, newReferenceBase: CPose3D) -> None:
        """
        Makes this = newReferenceBase (+) this, using the 2D part of a 3D reference pose.
        """
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
    def inverse(self) -> CPosePDFGaussian:
        """
        Returns the PDF of the inverse pose, (0,0,0) (-) this, as a new object. This PDF is not modified.
        """
    @typing.overload
    def inverseComposition(self, x: CPosePDFGaussian, ref: CPosePDFGaussian) -> None:
        """
        Sets this = x (-) ref, propagating both covariances (independent variables).
        """
    @typing.overload
    def inverseComposition(self, x1: CPosePDFGaussian, x0: CPosePDFGaussian, COV_01: mrpt.math.CMatrixDouble33) -> None:
        """
        Sets this = x1 (-) x0, given the 3x3 cross-covariance COV_01 = E[(x0-m0)(x1-m1)^T].
        """
class CPosePDFGaussianInf(CPosePDF):
    """
    A PDF of a 2D pose as a Gaussian with a mean and a 3x3 information (inverse covariance) matrix.
    """
    cov_inv: mrpt.math.CMatrixDouble33
    mean: CPose2D
    def __copy__(self) -> CPosePDFGaussianInf:
        """
        Returns a copy of this object.
        """
    def __deepcopy__(self, memo: dict) -> CPosePDFGaussianInf:
        """
        Returns a copy of this object.
        """
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
    @typing.overload
    def __init__(self, other: CPosePDFGaussianInf) -> None:
        """
        Copy constructor.
        """
    def __repr__(self) -> str:
        ...
    def __str__(self) -> str:
        ...
    def drawSingleSample(self) -> CPose2D:
        """
        Draws a single sample from the distribution.
        """
class CPosePDFSOG(CPosePDF):
    """
    A PDF of a 2D pose as a Sum of Gaussians (SOG): a set of weighted Gaussian modes.
    """
    class TGaussianMode:
        """
        One mode of a CPosePDFSOG.
        """
        def __init__(self) -> None:
            """
            Default constructor.
            """
        def __repr__(self) -> str:
            ...
        @property
        def cov(self) -> mrpt.math.CMatrixDouble33:
            """
            3x3 covariance matrix.
            """
        @cov.setter
        def cov(self, arg0: mrpt.math.CMatrixDouble33) -> None:
            ...
        @property
        def log_w(self) -> float:
            """
            Logarithm of the weight of the mode.
            """
        @log_w.setter
        def log_w(self, arg0: float) -> None:
            ...
        @property
        def mean(self) -> CPose2D:
            """
            Mean of the mode (CPose2D).
            """
        @mean.setter
        def mean(self, arg0: CPose2D) -> None:
            ...
    def __copy__(self) -> CPosePDFSOG:
        """
        Returns a copy of this object.
        """
    def __deepcopy__(self, memo: dict) -> CPosePDFSOG:
        """
        Returns a copy of this object.
        """
    def __getitem__(self, arg0: int) -> CPosePDFSOG.TGaussianMode:
        ...
    @typing.overload
    def __init__(self, nModes: int = 1) -> None:
        """
        Creates the given number of modes.
        """
    @typing.overload
    def __init__(self, other: CPosePDFSOG) -> None:
        """
        Copy constructor.
        """
    def __iter__(self) -> typing.Iterator:
        ...
    def __len__(self) -> int:
        ...
    def __repr__(self) -> str:
        ...
    def clear(self) -> None:
        """
        Removes all the modes.
        """
    def empty(self) -> bool:
        """
        Returns true if there are no modes.
        """
    def evaluateNormalizedPDF(self, x: CPose2D) -> float:
        """
        Evaluates the ratio PDF(x) / max(PDF), in the range [0,1].
        """
    def evaluatePDF(self, x: CPose2D, sumOverAllPhis: bool = False) -> float:
        """
        Evaluates the PDF at a given pose.
        """
    def get(self, i: int) -> CPosePDFSOG.TGaussianMode:
        """
        Returns a reference to the i'th mode.
        """
    def getMostLikelyCovarianceAndMean(self) -> tuple:
        """
        Returns the tuple (cov: CMatrixDouble33, mean: CPose2D) of the mode with the highest weight.
        """
    def mergeModes(self, max_KLd: float = 0.5, verbose: bool = False) -> None:
        """
        Merges very close modes, up to a maximum Kullback-Leibler distance.
        """
    def normalizeWeights(self) -> None:
        """
        Normalizes the weights of the modes, so that they sum up to one.
        """
    def push_back(self, mode: CPosePDFSOG.TGaussianMode) -> None:
        """
        Appends a mode (TGaussianMode).
        """
    def resize(self, N: int) -> None:
        """
        Sets the number of modes.
        """
    def size(self) -> int:
        """
        Returns the number of modes.
        """
class TInterpolatorMethod:
    """
    Interpolation method of CPose2DInterpolator and CPose3DInterpolator. In 3D, all methods but the Slerp ones interpolate yaw, pitch and roll independently.
    
    Members:
    
      imSpline : Spline interpolation using 4 points (2 before and 2 after the query time).
    
      imLinear2Neig : Linear interpolation between the previous and next neighbors.
    
      imLinear4Neig : Linear fit of the 4 closest points (2 before and 2 after the query time).
    
      imSSLLLL : Spline for x and y, linear least squares for z, yaw, pitch and roll.
    
      imSSLSLL : Spline for x, y and yaw, linear least squares for z, pitch and roll.
    
      imLinearSlerp : Linear for x, y and z, Slerp for the rotation (the default).
    
      imSplineSlerp : Spline for x, y and z (as imSpline), Slerp for the rotation.
    """
    __members__: typing.ClassVar[dict[str, TInterpolatorMethod]]
    imLinear2Neig: typing.ClassVar[TInterpolatorMethod]
    imLinear4Neig: typing.ClassVar[TInterpolatorMethod]
    imLinearSlerp: typing.ClassVar[TInterpolatorMethod]
    imSSLLLL: typing.ClassVar[TInterpolatorMethod]
    imSSLSLL: typing.ClassVar[TInterpolatorMethod]
    imSpline: typing.ClassVar[TInterpolatorMethod]
    imSplineSlerp: typing.ClassVar[TInterpolatorMethod]
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
class CPose2DInterpolator(mrpt.serialization.CSerializable):
    """
    A time-stamped trajectory in SE(2), with interpolation between poses.
    """
    def __copy__(self) -> CPose2DInterpolator:
        """
        Returns a copy of this object.
        """
    def __deepcopy__(self, memo: dict) -> CPose2DInterpolator:
        """
        Returns a copy of this object.
        """
    @typing.overload
    def __init__(self) -> None:
        """
        Default constructor.
        """
    @typing.overload
    def __init__(self, other: CPose2DInterpolator) -> None:
        """
        Copy constructor.
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
    def getInterpolationMethod(self) -> TInterpolatorMethod:
        """
        Returns the current interpolation method.
        """
    def insert(self, arg0: datetime.timedelta, arg1: mrpt.math.TPose2D) -> None:
        """
        Inserts a new pose in the sequence. It overwrites any previously existing pose at exactly the same time.
        """
    def interpolate(self, arg0: datetime.timedelta) -> tuple:
        """
        Returns (TPose2D, valid): interpolated pose at given time
        """
    def setInterpolationMethod(self, method: TInterpolatorMethod) -> None:
        """
        Sets the interpolation method (default: imLinearSlerp).
        """
    def size(self) -> int:
        """
        Returns the number of poses in the trajectory.
        """
class CPose3DInterpolator(mrpt.serialization.CSerializable):
    """
    A time-stamped trajectory in SE(3), with interpolation between poses.
    """
    def __copy__(self) -> CPose3DInterpolator:
        """
        Returns a copy of this object.
        """
    def __deepcopy__(self, memo: dict) -> CPose3DInterpolator:
        """
        Returns a copy of this object.
        """
    @typing.overload
    def __init__(self) -> None:
        """
        Default constructor.
        """
    @typing.overload
    def __init__(self, other: CPose3DInterpolator) -> None:
        """
        Copy constructor.
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
    def getInterpolationMethod(self) -> TInterpolatorMethod:
        """
        Returns the current interpolation method.
        """
    def insert(self, arg0: datetime.timedelta, arg1: mrpt.math.TPose3D) -> None:
        """
        Inserts a new pose in the sequence. It overwrites any previously existing pose at exactly the same time.
        """
    def interpolate(self, arg0: datetime.timedelta) -> tuple:
        """
        Returns (TPose3D, valid): interpolated pose at given time
        """
    def setInterpolationMethod(self, method: TInterpolatorMethod) -> None:
        """
        Sets the interpolation method (default: imLinearSlerp).
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
imLinear2Neig: TInterpolatorMethod
imLinear4Neig: TInterpolatorMethod
imLinearSlerp: TInterpolatorMethod
imSSLLLL: TInterpolatorMethod
imSSLSLL: TInterpolatorMethod
imSpline: TInterpolatorMethod
imSplineSlerp: TInterpolatorMethod
