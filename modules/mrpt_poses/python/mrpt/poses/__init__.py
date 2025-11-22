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

# PDFs
CPosePDF = _b.CPosePDF
CPose3DPDF = _b.CPose3DPDF
CPose3DPDFGaussian = _b.CPose3DPDFGaussian
CPose3DPDFGaussianInf = _b.CPose3DPDFGaussianInf

# Averaging
SE_average2 = _b.SE_average2
SE_average3 = _b.SE_average3

# ----- Global Functions -----
# (None exported explicitly in this batch, but operators act as functions)

# ----- Optional __all__ -----
__all__ = [
    "CPose2D",
    "CPose3D",
    "CPosePDF",
    "CPose3DPDF",
    "CPose3DPDFGaussian",
    "CPose3DPDFGaussianInf",
    "SE_average2",
    "SE_average3",
]
