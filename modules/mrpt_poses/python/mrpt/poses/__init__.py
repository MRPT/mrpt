"""
mrpt-poses Python API.
"""

# Import the compiled pybind11 module
from . import _bindings as _b

# ----- Classes -----
# 2D Poses
CPose2D = _b.CPose2D

# 3D Poses
CPose3D = _b.CPose3D

# Quaternion pose
CPose3DQuat = _b.CPose3DQuat

# Points
CPoint2D = _b.CPoint2D
CPoint3D = _b.CPoint3D

# PDFs
CPosePDF = _b.CPosePDF
CPose3DPDF = _b.CPose3DPDF
CPose3DPDFGaussian = _b.CPose3DPDFGaussian
CPose3DPDFGaussianInf = _b.CPose3DPDFGaussianInf
CPosePDFGaussian = _b.CPosePDFGaussian
CPosePDFGaussianInf = _b.CPosePDFGaussianInf

# Interpolators
CPose2DInterpolator = _b.CPose2DInterpolator
CPose3DInterpolator = _b.CPose3DInterpolator

# Random sampling
CPoseRandomSampler = _b.CPoseRandomSampler

# Averaging
SE_average2 = _b.SE_average2
SE_average3 = _b.SE_average3

# ----- Optional __all__ -----
__all__ = [
    "CPose2D",
    "CPose3D",
    "CPose3DQuat",
    "CPoint2D",
    "CPoint3D",
    "CPosePDF",
    "CPose3DPDF",
    "CPose3DPDFGaussian",
    "CPose3DPDFGaussianInf",
    "CPosePDFGaussian",
    "CPosePDFGaussianInf",
    "CPose2DInterpolator",
    "CPose3DInterpolator",
    "CPoseRandomSampler",
    "SE_average2",
    "SE_average3",
]
