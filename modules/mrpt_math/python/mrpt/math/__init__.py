import numpy as np
from . import _bindings as _b

# --- Dynamic Matrices and Vectors ---
CMatrixDouble = _b.CMatrixDouble
CVectorDouble = _b.CVectorDouble

# --- Lightweight Math Types ---
TPoint2D = _b.TPoint2D
TPoint3D = _b.TPoint3D
TPoint2Df = _b.TPoint2Df
TPoint3Df = _b.TPoint3Df
TPose2D = _b.TPose2D
TPose3D = _b.TPose3D

# --- Geometry Primitives (Phase 0.1) ---
TSegment2D = _b.TSegment2D
TSegment3D = _b.TSegment3D
TLine2D = _b.TLine2D
TLine3D = _b.TLine3D
TPlane = _b.TPlane
TBoundingBox = _b.TBoundingBox
TBoundingBoxf = _b.TBoundingBoxf
TTwist2D = _b.TTwist2D
TTwist3D = _b.TTwist3D
TPose3DQuat = _b.TPose3DQuat
CPolygon = _b.CPolygon
CHistogram = _b.CHistogram

# --- Fixed Size Matrices and Vectors ---
# We use dir() to catch all the CMatrixDoubleXX and CVectorFixedDoubleX
# we registered via the template helper in C++
_fixed_types = [name for name in dir(_b) if name.startswith(
    ("CMatrixDouble", "CVectorFixedDouble"))]

for name in _fixed_types:
    globals()[name] = getattr(_b, name)

# --- Free functions ---
wrapToPi = _b.wrapToPi
wrapTo2Pi = _b.wrapTo2Pi

# --- List of all classes to patch with NumPy support ---
_math_classes = [
    CMatrixDouble, CVectorDouble,
    TPoint2D, TPoint3D,
    TPoint2Df, TPoint3Df,
    TPose2D, TPose3D
] + [globals()[name] for name in _fixed_types]

# --- Convenience: Automatic NumPy conversion ---


def _patch_numpy_support(cls):
    # For Matrices/Vectors, we use the as_numpy() we wrote in C++
    if hasattr(cls, "as_numpy"):
        cls.__array__ = lambda self, dtype=None: self.as_numpy()

    # For TPoint/TPose, we can create a numpy array from their members
    elif cls is TPoint2D:
        cls.__array__ = lambda self, dtype=None: np.array([self.x, self.y])
    elif cls is TPoint3D:
        cls.__array__ = lambda self, dtype=None: np.array(
            [self.x, self.y, self.z])
    elif cls is TPose2D:
        cls.__array__ = lambda self, dtype=None: np.array(
            [self.x, self.y, self.phi])
    elif cls is TPose3D:
        cls.__array__ = lambda self, dtype=None: np.array(
            [self.x, self.y, self.z, self.yaw, self.pitch, self.roll])

    return cls


for _cls in _math_classes:
    _patch_numpy_support(_cls)

# NumPy support for new types
TTwist2D.__array__ = lambda self, dtype=None: np.array([self.vx, self.vy, self.omega])
TTwist3D.__array__ = lambda self, dtype=None: np.array(
    [self.vx, self.vy, self.vz, self.wx, self.wy, self.wz])
TPose3DQuat.__array__ = lambda self, dtype=None: np.array(
    [self.x, self.y, self.z, self.qr, self.qx, self.qy, self.qz])

# Clean up temporary variables
del _patch_numpy_support, _cls, _math_classes, _fixed_types

__all__ = [
    # Matrices
    "CMatrixDouble", "CVectorDouble",
    # Points and Poses
    "TPoint2D", "TPoint3D", "TPoint2Df", "TPoint3Df",
    "TPose2D", "TPose3D",
    # Geometry primitives
    "TSegment2D", "TSegment3D",
    "TLine2D", "TLine3D",
    "TPlane",
    "TBoundingBox", "TBoundingBoxf",
    "TTwist2D", "TTwist3D",
    "TPose3DQuat",
    "CPolygon",
    "CHistogram",
    # Free functions
    "wrapToPi", "wrapTo2Pi",
]
