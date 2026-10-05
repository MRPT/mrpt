"""
Python bindings for mrpt::gui — GUI windows for 3D visualization
"""
from __future__ import annotations
import mrpt.img
import mrpt.viz
__all__: list[str] = ['CBaseGUIWindow', 'CDisplayWindow3D']
class CBaseGUIWindow:
    """
    The base class for GUI window classes based on wxWidgets.
    """
    def clearKeyHitFlag(self) -> None:
        """
        Assure that "keyHit" will return false until the next pushed key.
        """
    def isOpen(self) -> bool:
        """
        Returns false if the user has already closed the window.
        """
    def keyHit(self) -> bool:
        """
        Returns true if a key has been pushed, without blocking waiting for a new key being pushed.
        """
    def waitForKey(self, ignoreControlKeys: bool = True) -> int:
        """
        Waits until a key is pressed in the window. Returns the key code.
        """
class CDisplayWindow3D(CBaseGUIWindow):
    """
    A graphical user interface (GUI) for efficiently rendering 3D scenes in real-time.
    """
    @staticmethod
    def Create(windowCaption: str = '', width: int = 400, height: int = 300) -> CDisplayWindow3D:
        """
        Creates a new 3D window (same arguments as the constructor).
        """
    def __init__(self, windowCaption: str = '', width: int = 400, height: int = 300) -> None:
        """
        Creates and shows a new 3D window with the given caption and size.
        """
    def __repr__(self) -> str:
        ...
    def forceRepaint(self) -> None:
        """
        Repaints the window. forceRepaint, repaint and updateWindow are all aliases of the same method.
        """
    def get3DSceneAndLock(self) -> mrpt.viz.Scene:
        """
        Get locked 3D scene pointer. Call unlockAccess3DScene() when done.
        """
    def getCameraAzimuthDeg(self) -> float:
        """
        Returns the camera azimuth angle, in degrees.
        """
    def getCameraElevationDeg(self) -> float:
        """
        Returns the camera elevation angle, in degrees.
        """
    def getCameraPointingToPoint(self) -> tuple:
        """
        Returns the point the camera looks at, as (x, y, z).
        """
    def getCameraZoom(self) -> float:
        """
        Returns the camera distance to the point it looks at.
        """
    def getFOV(self) -> float:
        """
        Returns the camera field of view, in degrees.
        """
    def getRenderingFPS(self) -> float:
        """
        Get the average Frames Per Second (FPS) value from the last 250 rendering events.
        """
    def grabImagesStart(self, prefix: str = 'video_') -> None:
        """
        Starts saving each rendered frame as a PNG file, with the given filename prefix.
        """
    def grabImagesStop(self) -> None:
        """
        Stops image grabbing started by grabImagesStart.
        """
    def isCameraProjective(self) -> bool:
        """
        Returns true for a perspective camera, false for an orthographic one.
        """
    def repaint(self) -> None:
        """
        Repaints the window. forceRepaint, repaint and updateWindow are all aliases of the same method.
        """
    def resize(self, arg0: int, arg1: int) -> None:
        """
        Resizes the window, stretching the image to fit into the display area.
        """
    def setCameraAzimuthDeg(self, arg0: float) -> None:
        """
        Sets the camera azimuth angle, in degrees.
        """
    def setCameraElevationDeg(self, arg0: float) -> None:
        """
        Sets the camera elevation angle, in degrees.
        """
    def setCameraPointingToPoint(self, arg0: float, arg1: float, arg2: float) -> None:
        """
        Sets the point the camera looks at (x, y, z).
        """
    def setCameraZoom(self, arg0: float) -> None:
        """
        Sets the camera distance to the point it looks at.
        """
    def setFOV(self, arg0: float) -> None:
        """
        Sets the camera field of view, in degrees.
        """
    def setImageView(self, img: mrpt.img.CImage) -> None:
        """
        Display a 2D image in this window
        """
    def setMaxRange(self, arg0: float) -> None:
        """
        Sets the far clip distance of the camera.
        """
    def setMinRange(self, arg0: float) -> None:
        """
        Sets the near clip distance of the camera.
        """
    def setPos(self, arg0: int, arg1: int) -> None:
        """
        Changes the position of the window on the screen.
        """
    def setProjectiveModel(self, arg0: bool) -> None:
        """
        Sets a perspective (true) or orthographic (false) camera.
        """
    def setWindowTitle(self, arg0: str) -> None:
        """
        Changes the window title.
        """
    def unlockAccess3DScene(self) -> None:
        """
        Releases the scene lock taken by get3DSceneAndLock().
        """
    def updateWindow(self) -> None:
        """
        Repaints the window. forceRepaint, repaint and updateWindow are all aliases of the same method.
        """
    def useCameraFromScene(self, arg0: bool) -> None:
        """
        If true, the camera of the scene viewport is used instead of the mouse-controlled one.
        """
