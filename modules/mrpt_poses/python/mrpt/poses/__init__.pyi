"""
mrpt-poses Python API.
"""
from __future__ import annotations
import mrpt as mrpt
from mrpt.poses._bindings import CPoint2D as CPoint2D
from mrpt.poses._bindings import CPoint3D as CPoint3D
from mrpt.poses._bindings import CPose2D as CPose2D
from mrpt.poses._bindings import CPose2DInterpolator as CPose2DInterpolator
from mrpt.poses._bindings import CPose3D as CPose3D
from mrpt.poses._bindings import CPose3DInterpolator as CPose3DInterpolator
from mrpt.poses._bindings import CPose3DPDF as CPose3DPDF
from mrpt.poses._bindings import CPose3DPDFGaussian as CPose3DPDFGaussian
from mrpt.poses._bindings import CPose3DPDFGaussianInf as CPose3DPDFGaussianInf
from mrpt.poses._bindings import CPose3DPDFParticles as CPose3DPDFParticles
from mrpt.poses._bindings import CPose3DQuat as CPose3DQuat
from mrpt.poses._bindings import CPosePDF as CPosePDF
from mrpt.poses._bindings import CPosePDFGaussian as CPosePDFGaussian
from mrpt.poses._bindings import CPosePDFGaussianInf as CPosePDFGaussianInf
from mrpt.poses._bindings import CPosePDFParticles as CPosePDFParticles
from mrpt.poses._bindings import CPoseRandomSampler as CPoseRandomSampler
from mrpt.poses._bindings import SE_average2 as SE_average2
from mrpt.poses._bindings import SE_average3 as SE_average3
from . import _bindings
__all__: list = ['CPose2D', 'CPose3D', 'CPose3DQuat', 'CPoint2D', 'CPoint3D', 'CPosePDF', 'CPose3DPDF', 'CPose3DPDFGaussian', 'CPose3DPDFGaussianInf', 'CPosePDFGaussian', 'CPosePDFGaussianInf', 'CPosePDFParticles', 'CPose3DPDFParticles', 'CPose2DInterpolator', 'CPose3DInterpolator', 'CPoseRandomSampler', 'SE_average2', 'SE_average3']
