"""
Python bindings for mrpt-img
"""
from __future__ import annotations
import mrpt.math
import mrpt.serialization
import numpy
import typing
__all__: list[str] = ['CImage', 'DistortionModel', 'PixelDepth', 'TCamera', 'TColor', 'TColorf', 'TColormap', 'TPixelCoord', 'TPixelCoordf', 'TStereoCamera', 'cmGRAYSCALE', 'cmHOT', 'cmJET', 'cmNONE', 'colormap', 'kannala_brandt', 'none', 'plumb_bob']
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
    """
    An RGBA color, 8 bits per channel.
    """
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
        """
        Builds a color from its components (0-255).
        """
    @typing.overload
    def __init__(self, arg0: list[int]) -> None:
        """
        Builds a color from a list [r, g, b] or [r, g, b, a] (0-255).
        """
    def __ne__(self, arg0: TColor) -> bool:
        ...
    def __repr__(self) -> str:
        ...
class TColorf:
    """
    An RGBA color - floats in the range [0,1].
    """
    A: float
    B: float
    G: float
    R: float
    def __array__(self, dtype = None, **kw):
        ...
    @typing.overload
    def __init__(self) -> None:
        """
        Default constructor.
        """
    @typing.overload
    def __init__(self, r: float, g: float, b: float, alpha: float = 1.0) -> None:
        """
        Builds a color from its components (0-1).
        """
    @typing.overload
    def __init__(self, color: TColor) -> None:
        """
        Builds a float color from an 8-bit TColor.
        """
    def __repr__(self) -> str:
        ...
    def asTColor(self) -> TColor:
        """
        Converts to a TColor with uint8 components
        """
class TPixelCoord:
    """
    Integer pixel coordinates (x, y).
    """
    x: int
    y: int
    def __init__(self, arg0: int, arg1: int) -> None:
        """
        Builds a pixel coordinate from (x, y).
        """
class TPixelCoordf:
    """
    Sub-pixel coordinates (x, y), as floats.
    """
    x: float
    y: float
    def __init__(self, arg0: float, arg1: float) -> None:
        """
        Builds a pixel coordinate from (x, y).
        """
class PixelDepth:
    """
    Bit depth of each image channel
    
    Members:
    
      D8U
    
      D16U
    """
    D16U: typing.ClassVar[PixelDepth]
    D8U: typing.ClassVar[PixelDepth]
    __members__: typing.ClassVar[dict[str, PixelDepth]]
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
class CImage:
    """
    A class for storing images as grayscale, RGB, or RGBA bitmaps.
    """
    @staticmethod
    def from_numpy(array: numpy.ndarray) -> CImage:
        """
        Creates a CImage from a NumPy uint8 array of shape (H, W) or (H, W, C), with 1, 3 or 4 channels (the data is copied).
        """
    def __array__(self, dtype = None, **kw):
        ...
    @typing.overload
    def __init__(self) -> None:
        """
        Default constructor: an empty image.
        """
    @typing.overload
    def __init__(self, array: numpy.ndarray) -> None:
        """
        Builds an image from a NumPy uint8 array of shape (height, width) or (height, width, channels), with 1, 3 or 4 channels.
        """
    def as_numpy(self) -> numpy.ndarray:
        """
        Returns a zero-copy NumPy view of the image, of shape (height, width, channels) and dtype uint8 or uint16.
        """
    def drawCircle(self, center: TPixelCoord, radius: int, color: TColor, width: int = 1) -> None:
        """
        Draws a circle of a given radius.
        """
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
        True if the image has 3 or 4 channels (RGB or RGBA)
        """
    def loadFromFile(self, filename: str) -> bool:
        """
        Load image from file. Returns True on success.
        """
    def resize(self, width: int, height: int, channels: int = 3, depth: PixelDepth = ...) -> None:
        """
        Changes the image size and number of channels (1, 3 or 4), erasing its contents (it does not scale them).
        """
    def saveToFile(self, filename: str, jpeg_quality: int = 95) -> bool:
        """
        Save image to file. Returns True on success.
        """
    def textOut(self, p: TPixelCoord, str: str, color: TColor) -> None:
        """
        Renders 2D text using bitmap fonts.
        """
class TCamera:
    """
    Intrinsic parameters for a pinhole or fisheye camera model, along with the associated lens distortion model.
    """
    cx: float
    cy: float
    dist: list[float]
    distortion: DistortionModel
    fx: float
    fy: float
    ncols: int
    nrows: int
    def __init__(self) -> None:
        """
        Default constructor: all intrinsic parameters set to zero.
        """
    def __repr__(self) -> str:
        ...
    def intrinsicParams(self) -> list[list[float]]:
        """
        Return intrinsic matrix as 3x3 list of lists (use np.array() to convert).
        """
class TStereoCamera(mrpt.serialization.CSerializable):
    """
    Structure to hold the parameters of a pinhole stereo camera model.
    """
    leftCamera: TCamera
    rightCamera: TCamera
    rightCameraPose: mrpt.math.TPose3DQuat
    def __init__(self) -> None:
        """
        Default constructor.
        """
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
