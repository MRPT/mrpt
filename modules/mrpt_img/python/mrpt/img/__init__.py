"""
mrpt-img Python API.
"""

# Import the compiled pybind11 module
import numpy as np
from . import _bindings as _b

# Export Classes
CImage = _b.CImage
TColor = _b.TColor
TCamera = _b.TCamera
DistortionModel = _b.DistortionModel
TPixelCoord = _b.TPixelCoord

# 1. Patch CImage for seamless NumPy/OpenCV integration


def _CImage_array(self, dtype=None):
    return self.as_numpy()


CImage.__array__ = _CImage_array

# 2. Patch TColor for easy tuple/array conversion


def _TColor_array(self, dtype=None):
    return np.array([self.R, self.G, self.B, self.A], dtype=np.uint8)


TColor.__array__ = _TColor_array

# 3. Convenience: create color constants


class Color:
    RED = TColor(255, 0, 0)
    GREEN = TColor(0, 255, 0)
    BLUE = TColor(0, 0, 255)
    WHITE = TColor(255, 255, 255)
    BLACK = TColor(0, 0, 0)


# Export Functions
colormap = _b.colormap

__all__ = ['CImage', 'TColor', 'TCamera',
           'DistortionModel', 'Color', 'colormap']
