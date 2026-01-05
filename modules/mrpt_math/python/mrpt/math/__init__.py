import numpy as np
from . import _bindings as _b

# --- Dynamic Matrices and Vectors ---
CMatrixDouble = _b.CMatrixDouble
CVectorDouble = _b.CVectorDouble

# --- Lightweight Math Types (New) ---
TPoint2D = _b.TPoint2D
TPoint3D = _b.TPoint3D
TPoint2Df = _b.TPoint2Df
TPoint3Df = _b.TPoint3Df
TPose2D = _b.TPose2D
TPose3D = _b.TPose3D

# --- Fixed Size Matrices and Vectors ---
# We use dir() to catch all the CMatrixDoubleXX and CVectorFixedDoubleX
# we registered via the template helper in C++
_fixed_types = [name for name in dir(_b) if name.startswith(
    ("CMatrixDouble", "CVectorFixedDouble"))]

for name in _fixed_types:
    globals()[name] = getattr(_b, name)

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

# Clean up temporary variables
del _patch_numpy_support, _cls, _math_classes, _fixed_types

__all__ = [
    "CMatrixDouble", "CVectorDouble",
    "TPoint2D", "TPoint3D", "TPose2D", "TPose3D"
]
