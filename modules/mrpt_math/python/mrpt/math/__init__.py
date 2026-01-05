"""
mrpt-math Python API.
"""

import numpy as np
from . import _bindings as _b

# ----- Dynamic Matrices and Vectors -----
CMatrixDouble = _b.CMatrixDouble
CVectorDouble = _b.CVectorDouble

# ----- Fixed Size Matrices (Commonly used in MRPT) -----
CMatrixDouble22 = _b.CMatrixDouble22
CMatrixDouble33 = _b.CMatrixDouble33
CMatrixDouble66 = _b.CMatrixDouble66

# ----- Fixed Size Vectors (Commonly used in MRPT) -----
CVectorFixedDouble2 = _b.CVectorFixedDouble2
CVectorFixedDouble3 = _b.CVectorFixedDouble3
CVectorFixedDouble6 = _b.CVectorFixedDouble6

# --- Convenience: Automatic NumPy conversion ---
# This allows calling np.array(mrpt_matrix) directly.


def _patch_numpy_support(cls):
    if hasattr(cls, "as_numpy"):
        cls.__array__ = lambda self, dtype=None: self.as_numpy()
    return cls


# Apply the patch to all bound math classes
_math_classes = [
    CMatrixDouble, CVectorDouble,
    CMatrixDouble22, CMatrixDouble33, CMatrixDouble66,
    CVectorFixedDouble2, CVectorFixedDouble3, CVectorFixedDouble6
]

for _cls in _math_classes:
    _patch_numpy_support(_cls)

# Clean up namespace
del _patch_numpy_support, _cls, _math_classes

# ----- Module Metadata -----
__all__ = [
    "CMatrixDouble",
    "CVectorDouble",
    "CMatrixDouble22",
    "CMatrixDouble33",
    "CMatrixDouble66",
    "CVectorFixedDouble2",
    "CVectorFixedDouble3",
    "CVectorFixedDouble6",
]
