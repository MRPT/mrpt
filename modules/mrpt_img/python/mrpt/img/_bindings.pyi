"""
Python bindings for mrpt-img
"""
from __future__ import annotations
import mrpt.serialization
import numpy
import typing
__all__: list[str] = ['CImage', 'DistortionModel', 'TCamera', 'TColor', 'TColorf', 'TColormap', 'TPixelCoord', 'TPixelCoordf', 'TStereoCamera', 'cmGRAYSCALE', 'cmHOT', 'cmJET', 'cmNONE', 'colormap', 'kannala_brandt', 'none', 'plumb_bob']
class DistortionModel:
    """
    Members:
    
      none
    
      plumb_bob
    
      kannala_brandt
    """
    __members__: typing.ClassVar[dict[str, DistortionModel]]
    kannala_brandt: typing.ClassVar[DistortionModel]
    none: typing.ClassVar[DistortionModel]
    plumb_bob: typing.ClassVar[DistortionModel]
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
class TColor:
    __hash__: typing.ClassVar[None] = None
    A: int
    B: int
    G: int
    R: int
    def __array__(self, dtype = None, **kw):
        ...
    def __eq__(self, arg0: TColor) -> bool:
        ...
    @typing.overload
    def __init__(self, r: int, g: int, b: int, alpha: int = 255) -> None:
        ...
    @typing.overload
    def __init__(self, arg0: list[int]) -> None:
        ...
    def __ne__(self, arg0: TColor) -> bool:
        ...
    def __repr__(self) -> str:
        ...
class TColorf:
    A: float
    B: float
    G: float
    R: float
    def __array__(self, dtype = None, **kw):
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, r: float, g: float, b: float, alpha: float = 1.0) -> None:
        ...
    @typing.overload
    def __init__(self, color: TColor) -> None:
        ...
    def __repr__(self) -> str:
        ...
    def asTColor(self) -> TColor:
        """
        Converts to a TColor with uint8 components
        """
class TPixelCoord:
    x: int
    y: int
    def __init__(self, arg0: int, arg1: int) -> None:
        ...
class TPixelCoordf:
    x: float
    y: float
    def __init__(self, arg0: float, arg1: float) -> None:
        ...
class CImage:
    @staticmethod
    def from_numpy(array: numpy.ndarray) -> CImage:
        """
        Create a CImage from a HxWxC numpy uint8 array (zero-copy not used).
        """
    def __array__(self, dtype = None, **kw):
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, arg0: numpy.ndarray) -> None:
        ...
    def as_numpy(self) -> numpy.ndarray:
        """
        Returns a Zero-Copy NumPy view of the image data.
        """
    def drawCircle(self, center: TPixelCoord, radius: int, color: TColor, width: int = 1) -> None:
        ...
    def getHeight(self) -> int:
        """
        Image height in pixels
        """
    def getWidth(self) -> int:
        """
        Image width in pixels
        """
    def isColor(self) -> bool:
        """
        True if the image has 3 channels (RGB)
        """
    def loadFromFile(self, filename: str) -> bool:
        """
        Load image from file. Returns True on success.
        """
    def resize(self, arg0: int, arg1: int, arg2: typing.Any, arg3: typing.Any) -> None:  # unnamed C++ type
        ...
    def saveToFile(self, filename: str, jpeg_quality: int = 95) -> bool:
        """
        Save image to file. Returns True on success.
        """
    def textOut(self, p: TPixelCoord, str: str, color: TColor) -> None:
        ...
class TCamera:
    cx: float
    cy: float
    dist: list[float]
    distortion: DistortionModel
    fx: float
    fy: float
    ncols: int
    nrows: int
    def __init__(self) -> None:
        ...
    def __repr__(self) -> str:
        ...
    def intrinsicParams(self) -> list[list[float]]:
        """
        Return intrinsic matrix as 3x3 list of lists (use np.array() to convert).
        """
class TStereoCamera(mrpt.serialization.CSerializable):
    leftCamera: TCamera
    rightCamera: TCamera
    rightCameraPose: typing.Any  # unnamed C++ type
    def __init__(self) -> None:
        ...
    def __repr__(self) -> str:
        ...
class TColormap:
    """
    Members:
    
      cmNONE
    
      cmGRAYSCALE
    
      cmJET
    
      cmHOT
    """
    __members__: typing.ClassVar[dict[str, TColormap]]
    cmGRAYSCALE: typing.ClassVar[TColormap]
    cmHOT: typing.ClassVar[TColormap]
    cmJET: typing.ClassVar[TColormap]
    cmNONE: typing.ClassVar[TColormap]
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
def colormap(color_map: TColormap, color_index: float) -> TColorf:
    """
    Maps a value in [0,1] to a TColorf using the given colormap
    """
cmGRAYSCALE: TColormap
cmHOT: TColormap
cmJET: TColormap
cmNONE: TColormap
kannala_brandt: DistortionModel
none: DistortionModel
plumb_bob: DistortionModel
