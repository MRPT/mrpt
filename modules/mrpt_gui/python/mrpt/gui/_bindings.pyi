"""
Python bindings for mrpt::gui — GUI windows for 3D visualization
"""
from __future__ import annotations
import mrpt.img
import mrpt.viz
import typing
__all__: list[str] = ['CBaseGUIWindow', 'CDisplayWindow3D', 'CDisplayWindowGUI', 'CDisplayWindowGUI_Params', 'HAS_NANOGUI', 'HAS_WXWIDGETS', 'nanogui_leave', 'nanogui_mainloop']
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
class CDisplayWindowGUI_Params:
    """
    Additional parameters for CDisplayWindowGUI windows.
    """
    fullscreen: bool
    maximized: bool
    resizable: bool
    def __init__(self) -> None:
        ...
    @property
    def nSamples(self) -> int:
        """
        Number of MSAA samples (0: no antialiasing).
        """
    @nSamples.setter
    def nSamples(self, arg0: int) -> None:
        ...
class CDisplayWindowGUI:
    """
    A window with a background 3D scene and mouse camera control, based on nanogui.
    
    Set the scene to show in `background_scene`. Then, either call spinOnce() from your
    own loop (it does not block), or register callbacks with addLoopCallback() and call
    mrpt.gui.nanogui_mainloop(), which blocks until all windows are closed.
    """
    def __init__(self, caption: str = '', width: int = 800, height: int = 600, params: CDisplayWindowGUI_Params = ...) -> None:
        """
        Creates and shows a new window.
        """
    def __repr__(self) -> str:
        ...
    def addDropFilesCallback(self, callback: typing.Callable) -> None:
        """
        Adds a handler f(filenames) -> bool for files dropped onto the window.
        """
    def addKeyboardCallback(self, callback: typing.Callable) -> None:
        """
        Adds a keyboard handler f(key, scancode, action, modifiers) -> bool (True if handled).
        Key codes are GLFW key codes (e.g. ord('A') for letters, 256 for ESC).
        """
    def addLoopCallback(self, callback: typing.Callable) -> None:
        """
        Adds a function to call before each repaint of the window.
        """
    def camera(self) -> mrpt.viz.COrbitCameraController:
        """
        The mouse-controlled camera (a mrpt.viz.COrbitCameraController).
        """
    def isOpen(self) -> bool:
        """
        Returns False if the window has been closed.
        """
    def resize(self, width: int, height: int) -> None:
        """
        Resizes the window.
        """
    def setIcon(self, img: mrpt.img.CImage) -> None:
        """
        Sets the window icon from an RGB or RGBA image.
        """
    def setPos(self, x: int, y: int) -> None:
        """
        Changes the position of the window on the screen.
        """
    def setVisible(self, visible: bool) -> None:
        """
        Shows or hides the window.
        """
    def setWindowTitle(self, title: str) -> None:
        """
        Changes the window title.
        """
    def spinOnce(self) -> bool:
        """
        Processes pending window events and repaints the window, without blocking.
        Returns False once the window has been closed.
        """
    @property
    def background_scene(self) -> mrpt.viz.Scene:
        """
        The 3D scene rendered as the window background (None: no scene).
        """
    @background_scene.setter
    def background_scene(self, arg1: mrpt.viz.Scene) -> None:
        ...
def nanogui_leave() -> None:
    """
    Makes nanogui_mainloop() return (e.g. call it from a callback).
    """
def nanogui_mainloop(idle_loop_period_ms: int = 50, min_repaint_period_ms: int = 50) -> None:
    """
    Runs the GUI loop of all CDisplayWindowGUI windows until they are all closed, or
    nanogui_leave() is called. Loop callbacks run before each repaint.
    """
HAS_NANOGUI: bool
HAS_WXWIDGETS: bool
