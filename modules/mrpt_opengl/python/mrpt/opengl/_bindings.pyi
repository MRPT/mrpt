from __future__ import annotations
import mrpt.img
import mrpt.viz
import numpy
__all__: list[str] = ['CFBORender', 'CFBORenderParameters']
class CFBORenderParameters:
    """
    Parameters for CFBORender constructor.
    """
    contextDebug: bool
    contextMajorVersion: int
    contextMinorVersion: int
    create_EGL_context: bool
    deviceIndexToUse: int
    height: int
    raw_depth: bool
    width: int
    def __init__(self, width: int = 800, height: int = 600) -> None:
        """
        Builds the parameters for the given image size.
        """
class CFBORender:
    """
    Render 3D scenes off-screen directly to RGB and/or RGB+D images.
    """
    def __init__(self, width: int = 800, height: int = 600) -> None:
        """
        Convenience constructor with just dimensions.
        """
    def clearCameraOverride(self) -> None:
        """
        Clear any camera override, reverting to using the scene's viewport camera.
        """
    def hasCameraOverride(self) -> bool:
        """
        Returns true if a camera override is set.
        """
    def height(self) -> int:
        """
        Returns the current render height in pixels.
        """
    def invalidateCompiledScene(self) -> None:
        """
        Force recompilation of the scene on next render.
        """
    def render_RGB(self, scene: mrpt.viz.Scene) -> mrpt.img.CImage:
        """
        Render scene to an RGB CImage
        """
    def render_RGBD(self, scene: mrpt.viz.Scene) -> tuple[mrpt.img.CImage, numpy.ndarray]:
        """
        Render scene to a tuple (RGB CImage, depth float32 NumPy array)
        """
    def render_depth(self, scene: mrpt.viz.Scene) -> numpy.ndarray:
        """
        Render scene to a depth map: a float32 NumPy array (rows, cols)
        """
    def setCamera(self, camera: mrpt.viz.CCamera) -> None:
        """
        Set the camera to use for rendering, overriding the scene's viewport camera.
        """
    def width(self) -> int:
        """
        Returns the current render width in pixels.
        """
