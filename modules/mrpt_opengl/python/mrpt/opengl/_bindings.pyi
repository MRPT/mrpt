from __future__ import annotations
import typing
import mrpt.img
import mrpt.viz
__all__: list[str] = ['CFBORender', 'CFBORenderParameters']
class CFBORenderParameters:
    contextDebug: bool
    contextMajorVersion: int
    contextMinorVersion: int
    create_EGL_context: bool
    deviceIndexToUse: int
    height: int
    raw_depth: bool
    width: int
    def __init__(self, width: int = 800, height: int = 600) -> None:
        ...
class CFBORender:
    def __init__(self, width: int = 800, height: int = 600) -> None:
        ...
    def clearCameraOverride(self) -> None:
        ...
    def hasCameraOverride(self) -> bool:
        ...
    def height(self) -> int:
        ...
    def invalidateCompiledScene(self) -> None:
        ...
    def render_RGB(self, scene: mrpt.viz.Scene) -> mrpt.img.CImage:
        """
        Render scene to an RGB CImage
        """
    def render_RGBD(self, scene: mrpt.viz.Scene) -> tuple:
        """
        Render scene to (RGB CImage, depth CMatrixFloat)
        """
    def render_depth(self, scene: mrpt.viz.Scene) -> typing.Any:  # unnamed C++ type
        """
        Render scene to a depth map (float matrix)
        """
    def setCamera(self, camera: mrpt.viz.CCamera) -> None:
        ...
    def width(self) -> int:
        ...
