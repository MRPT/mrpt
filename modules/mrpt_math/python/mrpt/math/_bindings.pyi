"""
Python bindings for mrpt_math (with NumPy support)
"""
from __future__ import annotations
import mrpt.serialization
import numpy
import typing
__all__: list[str] = ['CHistogram', 'CMatrixDouble', 'CMatrixDouble22', 'CMatrixDouble33', 'CMatrixDouble44', 'CMatrixDouble66', 'CMatrixDouble77', 'CPolygon', 'CQuaternionDouble', 'CVectorDouble', 'CVectorFixedDouble2', 'CVectorFixedDouble3', 'CVectorFixedDouble6', 'TBoundingBox', 'TBoundingBoxf', 'TLine2D', 'TLine3D', 'TPlane', 'TPoint2D', 'TPoint2Df', 'TPoint3D', 'TPoint3Df', 'TPose2D', 'TPose3D', 'TPose3DQuat', 'TSegment2D', 'TSegment3D', 'TTwist2D', 'TTwist3D', 'wrapTo2Pi', 'wrapToPi']
class CMatrixDouble:
    """
    A dynamic-size matrix of doubles, convertible to and from NumPy arrays.
    """
    def __array__(self, dtype = None, **kw):
        ...
    @typing.overload
    def __init__(self) -> None:
        """
        Default constructor.
        """
    @typing.overload
    def __init__(self, arg0: int, arg1: int) -> None:
        """
        Builds a matrix of the given size (rows, cols).
        """
    @typing.overload
    def __init__(self, arg0: numpy.ndarray) -> None:
        """
        Builds the matrix from a 2D NumPy array.
        """
    def __repr__(self) -> str:
        ...
    def as_numpy(self) -> numpy.ndarray:
        """
        Returns the matrix as a NumPy array.
        """
class CVectorDouble:
    """
    A dynamic-size vector of doubles, convertible to and from NumPy arrays.
    """
    def __array__(self, dtype = None, **kw):
        ...
    @typing.overload
    def __init__(self) -> None:
        """
        Default constructor.
        """
    @typing.overload
    def __init__(self, arg0: int) -> None:
        """
        Builds a vector of the given length.
        """
    @typing.overload
    def __init__(self, arg0: numpy.ndarray) -> None:
        """
        Builds the vector from a 1D NumPy array.
        """
    def as_numpy(self) -> numpy.ndarray:
        """
        Returns the vector as a NumPy array.
        """
class CMatrixDouble22:
    """
    2x2 fixed-size matrix of doubles, convertible to and from NumPy arrays.
    """
    def __array__(self, dtype = None, **kw):
        ...
    @typing.overload
    def __init__(self) -> None:
        """
        Builds a matrix filled with zeros.
        """
    @typing.overload
    def __init__(self, array: numpy.ndarray) -> None:
        """
        Builds the matrix from a NumPy array of the same shape
        """
    def __repr__(self) -> str:
        ...
    def as_numpy(self) -> numpy.ndarray:
        """
        Returns the matrix as a NumPy array.
        """
class CMatrixDouble33:
    """
    3x3 fixed-size matrix of doubles, convertible to and from NumPy arrays.
    """
    def __array__(self, dtype = None, **kw):
        ...
    @typing.overload
    def __init__(self) -> None:
        """
        Builds a matrix filled with zeros.
        """
    @typing.overload
    def __init__(self, array: numpy.ndarray) -> None:
        """
        Builds the matrix from a NumPy array of the same shape
        """
    def __repr__(self) -> str:
        ...
    def as_numpy(self) -> numpy.ndarray:
        """
        Returns the matrix as a NumPy array.
        """
class CMatrixDouble44:
    """
    4x4 fixed-size matrix of doubles, convertible to and from NumPy arrays.
    """
    def __array__(self, dtype = None, **kw):
        ...
    @typing.overload
    def __init__(self) -> None:
        """
        Builds a matrix filled with zeros.
        """
    @typing.overload
    def __init__(self, array: numpy.ndarray) -> None:
        """
        Builds the matrix from a NumPy array of the same shape
        """
    def __repr__(self) -> str:
        ...
    def as_numpy(self) -> numpy.ndarray:
        """
        Returns the matrix as a NumPy array.
        """
class CMatrixDouble66:
    """
    6x6 fixed-size matrix of doubles, convertible to and from NumPy arrays.
    """
    def __array__(self, dtype = None, **kw):
        ...
    @typing.overload
    def __init__(self) -> None:
        """
        Builds a matrix filled with zeros.
        """
    @typing.overload
    def __init__(self, array: numpy.ndarray) -> None:
        """
        Builds the matrix from a NumPy array of the same shape
        """
    def __repr__(self) -> str:
        ...
    def as_numpy(self) -> numpy.ndarray:
        """
        Returns the matrix as a NumPy array.
        """
class CMatrixDouble77:
    """
    7x7 fixed-size matrix of doubles, convertible to and from NumPy arrays.
    """
    def __array__(self, dtype = None, **kw):
        ...
    @typing.overload
    def __init__(self) -> None:
        """
        Builds a matrix filled with zeros.
        """
    @typing.overload
    def __init__(self, array: numpy.ndarray) -> None:
        """
        Builds the matrix from a NumPy array of the same shape
        """
    def __repr__(self) -> str:
        ...
    def as_numpy(self) -> numpy.ndarray:
        """
        Returns the matrix as a NumPy array.
        """
class CVectorFixedDouble2:
    """
    2x1 fixed-size matrix of doubles, convertible to and from NumPy arrays.
    """
    def __array__(self, dtype = None, **kw):
        ...
    @typing.overload
    def __init__(self) -> None:
        """
        Builds a matrix filled with zeros.
        """
    @typing.overload
    def __init__(self, array: numpy.ndarray) -> None:
        """
        Builds the matrix from a NumPy array of the same shape
        """
    def __repr__(self) -> str:
        ...
    def as_numpy(self) -> numpy.ndarray:
        """
        Returns the matrix as a NumPy array.
        """
class CVectorFixedDouble3:
    """
    3x1 fixed-size matrix of doubles, convertible to and from NumPy arrays.
    """
    def __array__(self, dtype = None, **kw):
        ...
    @typing.overload
    def __init__(self) -> None:
        """
        Builds a matrix filled with zeros.
        """
    @typing.overload
    def __init__(self, array: numpy.ndarray) -> None:
        """
        Builds the matrix from a NumPy array of the same shape
        """
    def __repr__(self) -> str:
        ...
    def as_numpy(self) -> numpy.ndarray:
        """
        Returns the matrix as a NumPy array.
        """
class CVectorFixedDouble6:
    """
    6x1 fixed-size matrix of doubles, convertible to and from NumPy arrays.
    """
    def __array__(self, dtype = None, **kw):
        ...
    @typing.overload
    def __init__(self) -> None:
        """
        Builds a matrix filled with zeros.
        """
    @typing.overload
    def __init__(self, array: numpy.ndarray) -> None:
        """
        Builds the matrix from a NumPy array of the same shape
        """
    def __repr__(self) -> str:
        ...
    def as_numpy(self) -> numpy.ndarray:
        """
        Returns the matrix as a NumPy array.
        """
class CQuaternionDouble:
    """
    A unit quaternion (r, x, y, z) for 3D rotations, with r the real part.
    """
    @typing.overload
    def __init__(self) -> None:
        """
        Identity rotation (1, 0, 0, 0).
        """
    @typing.overload
    def __init__(self, r: float, x: float, y: float, z: float) -> None:
        """
        Builds the quaternion from its components (it must be normalized).
        """
    def __mul__(self, arg0: CQuaternionDouble) -> CQuaternionDouble:
        ...
    def __repr__(self) -> str:
        ...
    def as_numpy(self) -> numpy.ndarray:
        """
        Returns the components [r, x, y, z] as a NumPy array.
        """
    def conj(self) -> CQuaternionDouble:
        """
        Returns the conjugate quaternion (the inverse rotation).
        """
    def ensurePositiveRealPart(self) -> None:
        """
        Flips the sign of all components if needed, so that r >= 0 (same rotation).
        """
    def normSqr(self) -> float:
        """
        Squared norm of the quaternion.
        """
    def normalize(self) -> None:
        """
        Normalizes the quaternion to unit norm.
        """
    def rotatePoint(self, x: float, y: float, z: float) -> tuple:
        """
        Rotates a 3D point; returns (x, y, z).
        """
    def rotationMatrix(self) -> CMatrixDouble33:
        """
        Returns the equivalent 3x3 rotation matrix.
        """
    def rpy(self) -> tuple:
        """
        Returns the equivalent (roll, pitch, yaw) angles, in radians.
        """
    @property
    def r(self) -> float:
        """
        Real part
        """
    @r.setter
    def r(self, arg1: float) -> None:
        ...
    @property
    def x(self) -> float:
        """
        Imaginary part, i
        """
    @x.setter
    def x(self, arg1: float) -> None:
        ...
    @property
    def y(self) -> float:
        """
        Imaginary part, j
        """
    @y.setter
    def y(self, arg1: float) -> None:
        ...
    @property
    def z(self) -> float:
        """
        Imaginary part, k
        """
    @z.setter
    def z(self, arg1: float) -> None:
        ...
class TPoint2Df:
    """
    A 2D point (x, y), with float coordinates.
    """
    __hash__: typing.ClassVar[None] = None
    x: float
    y: float
    def __add__(self, arg0: TPoint2Df) -> TPoint2Df:
        ...
    def __array__(self, dtype: typing.Any = None, copy: typing.Any = None) -> numpy.ndarray:
        ...
    def __eq__(self, arg0: TPoint2Df) -> bool:
        ...
    @typing.overload
    def __init__(self) -> None:
        """
        Default constructor. Initializes to zeros.
        """
    @typing.overload
    def __init__(self, arg0: float, arg1: float) -> None:
        """
        Constructor from coordinates.
        """
    @typing.overload
    def __init__(self, arg0: list[float]) -> None:
        """
        Builds the point from a list [x, y].
        """
    def __len__(self) -> int:
        ...
    def __ne__(self, arg0: TPoint2Df) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __str__(self) -> str:
        ...
    def __sub__(self, arg0: TPoint2Df) -> TPoint2Df:
        ...
    def cast_double(self) -> TPoint2D:
        """
        Returns a copy with double coordinates (TPoint2D).
        """
    def norm(self) -> float:
        """
        Returns the norm sqrt(x^2 + y^2).
        """
    def sqrNorm(self) -> float:
        """
        Returns the squared norm x^2 + y^2.
        """
    def unitarize(self) -> TPoint2Df:
        """
        Returns this vector with unit length: v/norm(v)
        """
class TPoint3Df:
    """
    A 3D point (x, y, z), with float coordinates.
    """
    __hash__: typing.ClassVar[None] = None
    x: float
    y: float
    z: float
    def __add__(self, arg0: TPoint3Df) -> TPoint3Df:
        ...
    def __array__(self, dtype: typing.Any = None, copy: typing.Any = None) -> numpy.ndarray:
        ...
    def __eq__(self, arg0: TPoint3Df) -> bool:
        ...
    @typing.overload
    def __init__(self) -> None:
        """
        Default constructor. Initializes to zeros.
        """
    @typing.overload
    def __init__(self, arg0: float, arg1: float, arg2: float) -> None:
        """
        Constructor from coordinates.
        """
    @typing.overload
    def __init__(self, arg0: list[float]) -> None:
        """
        Builds the point from a list [x, y, z].
        """
    def __len__(self) -> int:
        ...
    def __ne__(self, arg0: TPoint3Df) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __str__(self) -> str:
        ...
    def __sub__(self, arg0: TPoint3Df) -> TPoint3Df:
        ...
    def cast_double(self) -> TPoint3D:
        """
        Returns a copy with double coordinates (TPoint3D).
        """
    def cross(self, arg0: TPoint3Df) -> TPoint3Df:
        """
        Cross product res = cross(this, p)
        """
    def dot(self, arg0: TPoint3Df) -> float:
        """
        Scalar product s=dot(this,p)
        """
    def norm(self) -> float:
        """
        Returns the norm sqrt(x^2 + y^2 + z^2).
        """
    def sqrNorm(self) -> float:
        """
        Returns the squared norm x^2 + y^2 + z^2.
        """
    def unitarize(self) -> TPoint3Df:
        """
        Returns this vector with unit length: v/norm(v)
        """
class TPoint2D:
    """
    A 2D point (x, y), with double coordinates.
    """
    __hash__: typing.ClassVar[None] = None
    x: float
    y: float
    def __add__(self, arg0: TPoint2D) -> TPoint2D:
        ...
    def __array__(self, dtype = None, **kw):
        ...
    def __eq__(self, arg0: TPoint2D) -> bool:
        ...
    def __iadd__(self, arg0: TPoint2D) -> TPoint2D:
        ...
    @typing.overload
    def __init__(self) -> None:
        """
        Default constructor. Initializes to zeros.
        """
    @typing.overload
    def __init__(self, arg0: float, arg1: float) -> None:
        """
        Constructor from coordinates.
        """
    @typing.overload
    def __init__(self, arg0: list[float]) -> None:
        """
        Builds the point from a list [x, y].
        """
    def __isub__(self, arg0: TPoint2D) -> TPoint2D:
        ...
    def __len__(self) -> int:
        ...
    def __mul__(self, arg0: float) -> TPoint2D:
        ...
    def __ne__(self, arg0: TPoint2D) -> bool:
        ...
    def __neg__(self) -> TPoint2D:
        ...
    def __repr__(self) -> str:
        ...
    def __str__(self) -> str:
        ...
    def __sub__(self, arg0: TPoint2D) -> TPoint2D:
        ...
    def __truediv__(self, arg0: float) -> TPoint2D:
        ...
    def cast_float(self) -> TPoint2Df:
        """
        Returns a copy with float coordinates (TPoint2Df).
        """
    def norm(self) -> float:
        """
        Returns the norm sqrt(x^2 + y^2).
        """
    def sqrNorm(self) -> float:
        """
        Returns the squared norm x^2 + y^2.
        """
    def unitarize(self) -> TPoint2D:
        """
        Returns this vector with unit length: v/norm(v)
        """
class TPoint3D:
    """
    A 3D point (x, y, z), with double coordinates.
    """
    __hash__: typing.ClassVar[None] = None
    x: float
    y: float
    z: float
    def __add__(self, arg0: TPoint3D) -> TPoint3D:
        ...
    def __array__(self, dtype = None, **kw):
        ...
    def __eq__(self, arg0: TPoint3D) -> bool:
        ...
    def __iadd__(self, arg0: TPoint3D) -> TPoint3D:
        ...
    @typing.overload
    def __init__(self) -> None:
        """
        Default constructor. Initializes to zeros.
        """
    @typing.overload
    def __init__(self, arg0: float, arg1: float, arg2: float) -> None:
        """
        Constructor from coordinates.
        """
    @typing.overload
    def __init__(self, arg0: list[float]) -> None:
        """
        Builds the point from a list [x, y, z].
        """
    def __isub__(self, arg0: TPoint3D) -> TPoint3D:
        ...
    def __len__(self) -> int:
        ...
    def __mul__(self, arg0: float) -> TPoint3D:
        ...
    def __ne__(self, arg0: TPoint3D) -> bool:
        ...
    def __neg__(self) -> TPoint3D:
        ...
    def __repr__(self) -> str:
        ...
    def __str__(self) -> str:
        ...
    def __sub__(self, arg0: TPoint3D) -> TPoint3D:
        ...
    def __truediv__(self, arg0: float) -> TPoint3D:
        ...
    def cast_float(self) -> TPoint3Df:
        """
        Returns a copy with float coordinates (TPoint3Df).
        """
    def cross(self, arg0: TPoint3D) -> TPoint3D:
        """
        Cross product res = cross(this, p)
        """
    def dot(self, arg0: TPoint3D) -> float:
        """
        Scalar product s=dot(this,p)
        """
    def norm(self) -> float:
        """
        Returns the norm sqrt(x^2 + y^2 + z^2).
        """
    def sqrNorm(self) -> float:
        """
        Returns the squared norm x^2 + y^2 + z^2.
        """
    def unitarize(self) -> TPoint3D:
        """
        Returns this vector with unit length: v/norm(v)
        """
class TPose2D:
    """
    Lightweight 2D pose (x, y, phi): an element of SE(2).
    """
    __hash__: typing.ClassVar[None] = None
    phi: float
    x: float
    y: float
    def __add__(self, arg0: TPose2D) -> TPose2D:
        ...
    def __array__(self, dtype = None, **kw):
        ...
    def __eq__(self, arg0: TPose2D) -> bool:
        ...
    @typing.overload
    def __init__(self) -> None:
        """
        Default fast constructor. Initializes to zeros.
        """
    @typing.overload
    def __init__(self, arg0: float, arg1: float, arg2: float) -> None:
        """
        Constructor from coordinates.
        """
    @typing.overload
    def __init__(self, arg0: list[float]) -> None:
        """
        Builds the pose from a list [x, y, phi].
        """
    def __len__(self) -> int:
        ...
    def __ne__(self, arg0: TPose2D) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __str__(self) -> str:
        ...
    def __sub__(self, arg0: TPose2D) -> TPose2D:
        ...
    def composePoint(self, arg0: TPoint2D) -> TPoint2D:
        """
        Transforms a point from the local frame of this pose into the global (world) frame.
        """
    def inverseComposePoint(self, arg0: TPoint2D) -> TPoint2D:
        """
        Transforms a point from the global (world) frame into the local frame of this pose.
        """
    def norm(self) -> float:
        """
        Euclidean norm of the translation (x, y); phi is ignored.
        """
    def normalizePhi(self) -> None:
        """
        Wraps phi to the canonical range (-pi, pi].
        """
class TPose3D:
    """
    Lightweight 3D pose (x, y, z, yaw, pitch, roll): an element of SE(3).
    """
    __hash__: typing.ClassVar[None] = None
    pitch: float
    roll: float
    x: float
    y: float
    yaw: float
    z: float
    def __add__(self, arg0: TPose3D) -> TPose3D:
        ...
    def __array__(self, dtype = None, **kw):
        ...
    def __eq__(self, arg0: TPose3D) -> bool:
        ...
    @typing.overload
    def __init__(self) -> None:
        """
        Default fast constructor. Initializes to zeros.
        """
    @typing.overload
    def __init__(self, arg0: float, arg1: float, arg2: float, arg3: float, arg4: float, arg5: float) -> None:
        """
        Constructor from coordinates.
        """
    @typing.overload
    def __init__(self, arg0: list[float]) -> None:
        """
        Builds the pose from a list [x, y, z, yaw, pitch, roll].
        """
    def __len__(self) -> int:
        ...
    def __ne__(self, arg0: TPose3D) -> bool:
        ...
    def __neg__(self) -> TPose3D:
        ...
    def __repr__(self) -> str:
        ...
    def __str__(self) -> str:
        ...
    def composePoint(self, arg0: TPoint3D) -> TPoint3D:
        """
        Transforms a point from the local frame of this pose into the global (world) frame.
        """
    def getHomogeneousMatrix(self) -> CMatrixDouble44:
        """
        Returns the 4x4 homogeneous transformation matrix.
        """
    def getRotationMatrix(self) -> CMatrixDouble33:
        """
        Returns the 3x3 rotation matrix.
        """
    def inverseComposePoint(self, arg0: TPoint3D) -> TPoint3D:
        """
        Transforms a point from the global (world) frame into the local frame of this pose.
        """
    def norm(self) -> float:
        """
        Euclidean norm of the translation (x, y, z); angles are ignored.
        """
class TSegment2D:
    """
    2D segment, consisting of two points.
    """
    point1: TPoint2D
    point2: TPoint2D
    @staticmethod
    def FromPoints(arg0: TPoint2D, arg1: TPoint2D) -> TSegment2D:
        """
        Static method, returns segment from two points.
        """
    @typing.overload
    def __init__(self) -> None:
        """
        Fast default constructor. Initializes to (0,0)-(0,0)
        """
    @typing.overload
    def __init__(self, arg0: TPoint2D, arg1: TPoint2D) -> None:
        """
        Constructor from both points.
        """
    def __repr__(self) -> str:
        ...
    def contains(self, arg0: TPoint2D) -> bool:
        """
        Check whether a point is inside a segment.
        """
    def distance(self, arg0: TPoint2D) -> float:
        """
        Absolute distance to point.
        """
    def length(self) -> float:
        """
        Segment length.
        """
class TSegment3D:
    """
    3D segment, consisting of two points.
    """
    point1: TPoint3D
    point2: TPoint3D
    @typing.overload
    def __init__(self) -> None:
        """
        Fast default constructor. Initializes to (0,0,0)-(0,0,0)
        """
    @typing.overload
    def __init__(self, arg0: TPoint3D, arg1: TPoint3D) -> None:
        """
        Constructor from two points.
        """
    def __repr__(self) -> str:
        ...
    def contains(self, arg0: TPoint3D) -> bool:
        """
        Check whether a point is inside the segment.
        """
    def distance(self, arg0: TPoint3D) -> float:
        """
        Distance to point.
        """
    def length(self) -> float:
        """
        Segment length.
        """
class TLine2D:
    """
    2D line without bounds, represented by its equation Ax+By+C=0.
    """
    coefs: list[float]
    @staticmethod
    def FromTwoPoints(arg0: TPoint2D, arg1: TPoint2D) -> TLine2D:
        """
        Static constructor from two points.
        """
    @typing.overload
    def __init__(self) -> None:
        """
        Fast default constructor. Initializes to undefined values.
        """
    @typing.overload
    def __init__(self, arg0: TPoint2D, arg1: TPoint2D) -> None:
        """
        Constructor from two points, through which the line will pass.
        """
    @typing.overload
    def __init__(self, A: float, B: float, C: float) -> None:
        """
        Constructor from line's coefficients.
        """
    def __repr__(self) -> str:
        ...
    def __str__(self) -> str:
        ...
    def contains(self, arg0: TPoint2D) -> bool:
        """
        Check whether a point is inside the line.
        """
    def distance(self, arg0: TPoint2D) -> float:
        """
        Absolute distance from a given point.
        """
    def evaluatePoint(self, arg0: TPoint2D) -> float:
        """
        Evaluate point in the line's equation.
        """
    def unitarize(self) -> None:
        """
        Unitarize line's normal vector.
        """
class TLine3D:
    """
    3D line, represented by a base point and a director vector.
    """
    director: TPoint3D
    pBase: TPoint3D
    @staticmethod
    def FromTwoPoints(arg0: TPoint3D, arg1: TPoint3D) -> TLine3D:
        """
        Static constructor from two points.
        """
    @typing.overload
    def __init__(self) -> None:
        """
        Fast default constructor. Initializes to all zeros.
        """
    @typing.overload
    def __init__(self, arg0: TPoint3D, arg1: TPoint3D) -> None:
        """
        Constructor from two points, through which the line will pass.
        """
    def __repr__(self) -> str:
        ...
    def __str__(self) -> str:
        ...
    def closestPointTo(self, arg0: TPoint3D) -> TPoint3D:
        """
        Closest point to p along the line. It is computed as the intersection of this with the plane perpendicular to this that passes through p.
        """
    def contains(self, arg0: TPoint3D) -> bool:
        """
        Check whether a point is inside the line.
        """
    def distance(self, arg0: TPoint3D) -> float:
        """
        Absolute distance between the line and a point.
        """
    def unitarize(self) -> None:
        """
        Unitarize director vector.
        """
class TPlane:
    """
    3D Plane, represented by its equation Ax+By+Cz+D=0.
    """
    coefs: list[float]
    @staticmethod
    def From3Points(arg0: TPoint3D, arg1: TPoint3D, arg2: TPoint3D) -> TPlane:
        """
        Returns the plane that contains three points.
        """
    @typing.overload
    def __init__(self) -> None:
        """
        Fast default constructor (uninitialized coefficients).
        """
    @typing.overload
    def __init__(self, A: float, B: float, C: float, D: float) -> None:
        """
        Constructor from plane coefficients.
        """
    @typing.overload
    def __init__(self, arg0: TPoint3D, arg1: TPoint3D, arg2: TPoint3D) -> None:
        """
        Defines a plane which contains these three points.
        """
    def __repr__(self) -> str:
        ...
    def __str__(self) -> str:
        ...
    def contains(self, arg0: TPoint3D) -> bool:
        """
        Check whether a point is contained into the plane.
        """
    def distance(self, arg0: TPoint3D) -> float:
        """
        Absolute distance to 3D point.
        """
    def evaluatePoint(self, arg0: TPoint3D) -> float:
        """
        Evaluate a point in the plane's equation.
        """
    def unitarize(self) -> None:
        """
        Unitarize normal vector.
        """
class TBoundingBox:
    """
    A 3D axis-aligned bounding box, defined by its min and max corners.
    """
    max: TPoint3D
    min: TPoint3D
    @staticmethod
    def PlusMinusInfinity() -> TBoundingBox:
        """
        Initialize with min=+Infinity, max=-Infinity. This is useful as an initial value before processing a list of points to keep their minimum/maximum.
        """
    def __init__(self, arg0: TPoint3D, arg1: TPoint3D) -> None:
        """
        Builds the box from its min and max corners.
        """
    def __repr__(self) -> str:
        ...
    def containsPoint(self, arg0: TPoint3D) -> bool:
        """
        Returns true if the point lies within the bounding box (including the exact border)
        """
    def intersection(self, arg0: TBoundingBox) -> typing.Any:
        """
        Returns the intersection of this bounding box with "b", or std::nullopt if no intersection exists.
        """
    def unionWith(self, arg0: TBoundingBox) -> TBoundingBox:
        """
        Returns the union of this bounding box with "b", i.e. a new bounding box comprising both this and b.
        """
    def volume(self) -> float:
        """
        Returns the volume of the box.
        """
class TBoundingBoxf:
    """
    A 3D axis-aligned bounding box with float coordinates, defined by its min and max corners.
    """
    max: TPoint3Df
    min: TPoint3Df
    @staticmethod
    def PlusMinusInfinity() -> TBoundingBoxf:
        """
        Initialize with min=+Infinity, max=-Infinity. This is useful as an initial value before processing a list of points to keep their minimum/maximum.
        """
    def __init__(self, arg0: TPoint3Df, arg1: TPoint3Df) -> None:
        """
        Builds the box from its min and max corners.
        """
    def __repr__(self) -> str:
        ...
    def containsPoint(self, arg0: TPoint3Df) -> bool:
        """
        Returns true if the point lies within the bounding box (including the exact border)
        """
    def unionWith(self, arg0: TBoundingBoxf) -> TBoundingBoxf:
        """
        Returns the union of this bounding box with "b", i.e. a new bounding box comprising both this and b.
        """
    def volume(self) -> float:
        """
        Returns the volume of the box.
        """
class TTwist2D:
    """
    2D twist: 2D velocity vector (vx,vy) + planar angular velocity (omega)
    """
    omega: float
    vx: float
    vy: float
    def __array__(self, dtype = None, **kw):
        ...
    @typing.overload
    def __init__(self) -> None:
        """
        Default fast constructor. Initializes to zeros.
        """
    @typing.overload
    def __init__(self, vx: float, vy: float, omega: float) -> None:
        """
        Constructor from components.
        """
    def __len__(self) -> int:
        ...
    def __repr__(self) -> str:
        ...
    def __str__(self) -> str:
        ...
    def asString(self) -> str:
        """
        Returns a human-readable textual representation of the object (eg: "[vx vy omega]", omega in deg/s)
        """
    def rotate(self, arg0: float) -> None:
        """
        Transform the (vx,vy) components for a counterclockwise rotation of ang radians.
        """
    def rotated(self, arg0: float) -> TTwist2D:
        """
        Like rotate(), but returning a copy of the rotated twist.
        """
class TTwist3D:
    """
    3D twist: 3D velocity vector (vx,vy,vz) + angular velocity (wx,wy,wz)
    """
    vx: float
    vy: float
    vz: float
    wx: float
    wy: float
    wz: float
    def __array__(self, dtype = None, **kw):
        ...
    @typing.overload
    def __init__(self) -> None:
        """
        Default fast constructor. Initializes to zeros.
        """
    @typing.overload
    def __init__(self, vx: float, vy: float, vz: float, wx: float, wy: float, wz: float) -> None:
        """
        Constructor from components.
        """
    def __len__(self) -> int:
        ...
    def __repr__(self) -> str:
        ...
    def __str__(self) -> str:
        ...
    def asString(self) -> str:
        """
        Returns a human-readable textual representation of the object (eg: "[vx vy vz wx wy wz]", omegas in deg/s)
        """
    def rotate(self, arg0: TPose3D) -> None:
        """
        Transform all 6 components for a change of reference frame from "A" to another frame "B" whose rotation with respect to "A" is given by rot.
        """
    def rotated(self, arg0: TPose3D) -> TTwist3D:
        """
        Like rotate(), but returning a copy of the rotated twist.
        """
class TPose3DQuat:
    """
    Lightweight 3D pose (three spatial coordinates, plus a quaternion ).
    """
    qr: float
    qx: float
    qy: float
    qz: float
    x: float
    y: float
    z: float
    def __array__(self, dtype = None, **kw):
        ...
    @typing.overload
    def __init__(self) -> None:
        """
        Default fast constructor. Initializes to identity transformation.
        """
    @typing.overload
    def __init__(self, x: float, y: float, z: float, qr: float, qx: float, qy: float, qz: float) -> None:
        """
        Constructor from coordinates.
        """
    def __len__(self) -> int:
        ...
    def __repr__(self) -> str:
        ...
    def __str__(self) -> str:
        ...
class CPolygon(mrpt.serialization.CSerializable):
    """
    A 2D polygon, serializable.
    """
    def __init__(self) -> None:
        """
        Default constructor (empty polygon, 0 vertices)
        """
    def __len__(self) -> int:
        ...
    def __repr__(self) -> str:
        ...
    def add_vertex(self, x: float, y: float) -> None:
        """
        Add a new vertex to polygon.
        """
    def get_vertex_x(self, arg0: int) -> float:
        """
        Returns the x coordinate of the i-th vertex.
        """
    def get_vertex_y(self, arg0: int) -> float:
        """
        Returns the y coordinate of the i-th vertex.
        """
    def get_vertices(self) -> tuple:
        """
        Returns (xs, ys) as two lists of vertex coordinates
        """
    def set_vertices(self, arg0: list[float], arg1: list[float]) -> None:
        """
        Set all vertices at once.
        """
class CHistogram:
    """
    A histogram of a real-valued variable, with equally-sized bins.
    """
    def __init__(self, min: float, max: float, nBins: int) -> None:
        """
        Constructor.
        """
    def __repr__(self) -> str:
        ...
    def add(self, arg0: float) -> None:
        """
        Add an element to the histogram. If element is out of [min,max] it is ignored.
        """
    def clear(self) -> None:
        """
        Resets all bins to zero.
        """
    def getBinCount(self, arg0: int) -> int:
        """
        Returns the elements count into the selected bin index, where first one is 0.
        """
    def getBinRatio(self, arg0: int) -> float:
        """
        Returns the ratio in [0,1] range for the selected bin index, where first one is 0.
        """
    def getHistogram(self) -> tuple:
        """
        Returns (bin_centers, hit_counts) as two lists
        """
    def getHistogramNormalized(self) -> tuple:
        """
        Returns (bin_centers, normalized_hits) — integral equals 1 as PDF
        """
def wrapTo2Pi(arg0: float) -> float:
    """
    Wrap angle to [0, 2*pi]
    """
def wrapToPi(arg0: float) -> float:
    """
    Wrap angle to [-pi, pi]
    """
