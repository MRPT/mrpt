"""
mrpt-img Python API.
"""
from __future__ import annotations
import mrpt as mrpt
from mrpt.img._bindings import CImage as CImage
from mrpt.img._bindings import DistortionModel as DistortionModel
from mrpt.img._bindings import TCamera as TCamera
from mrpt.img._bindings import TColor as TColor
from mrpt.img._bindings import TColorf as TColorf
from mrpt.img._bindings import TColormap as TColormap
from mrpt.img._bindings import TPixelCoord as TPixelCoord
from mrpt.img._bindings import TPixelCoordf as TPixelCoordf
from mrpt.img._bindings import TStereoCamera as TStereoCamera
from mrpt.img._bindings import colormap as colormap
import numpy as np
import typing
from . import _bindings
__all__: list = ['CImage', 'TColor', 'TColorf', 'TCamera', 'TStereoCamera', 'DistortionModel', 'TPixelCoord', 'TPixelCoordf', 'Color', 'colormap', 'TColormap']
class Color:
    BLACK: typing.ClassVar[_bindings.TColor]
    BLUE: typing.ClassVar[_bindings.TColor]
    GREEN: typing.ClassVar[_bindings.TColor]
    RED: typing.ClassVar[_bindings.TColor]
    WHITE: typing.ClassVar[_bindings.TColor]
def _CImage_array(self, dtype = None, **kw):
    ...
def _TColor_array(self, dtype = None, **kw):
    ...
def _TColorf_array(self, dtype = None, **kw):
    ...
