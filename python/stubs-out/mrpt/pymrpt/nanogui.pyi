from typing import Any

from typing import overload
import mrpt.pymrpt

class Color:
    @overload
    def __init__(self) -> None: ...
    @overload
    def __init__(self, intensity: float, alpha: float) -> None: ...
    @overload
    def __init__(self, intensity: int, alpha: int) -> None: ...
    @overload
    def __init__(self, r: float, g: float, b: float, a: float) -> None: ...
    @overload
    def __init__(self, r: int, g: int, b: int, a: int) -> None: ...
    @overload
    def __init__(self, arg0: Color) -> None: ...
    def assign(self) -> Color: ...
    def b(self) -> float: ...
    def contrastingColor(self) -> Color: ...
    def g(self) -> float: ...
    def r(self) -> float: ...

class GLCanvas:
    @overload
    def __init__(self, arg0: GLCanvas) -> None: ...
    @overload
    def __init__(self, arg0: GLCanvas) -> None: ...
    def backgroundColor(self) -> Color: ...
    def draw(self, ctx: mrpt.pymrpt.NVGcontext) -> None: ...
    def drawBorder(self) -> bool: ...
    @overload
    def drawGL(self) -> None: ...
    @overload
    def drawGL() -> void: ...
    @overload
    def setBackgroundColor(self, backgroundColor: Color) -> None: ...
    @overload
    def setBackgroundColor(constclassnanogui) -> void: ...
    @overload
    def setDrawBorder(self, bDrawBorder: bool) -> None: ...
    @overload
    def setDrawBorder(constbool) -> void: ...

class Screen:
    @overload
    def __init__(self) -> None: ...
    @overload
    def __init__(self, arg0: Screen) -> None: ...
    @overload
    def __init__(self, arg0: Screen) -> None: ...
    def background(self) -> Color: ...
    def caption(self) -> str: ...
    @overload
    def centerWindow(self, window) -> None: ...
    @overload
    def centerWindow(classnanogui) -> void: ...
    @overload
    def charCallbackEvent(self, codepoint: int) -> bool: ...
    @overload
    def charCallbackEvent(unsignedint) -> bool: ...
    def cursorPosCallbackEvent(self, x: float, y: float) -> bool: ...
    @overload
    def disposeWindow(self, window) -> None: ...
    @overload
    def disposeWindow(classnanogui) -> void: ...
    @overload
    def drawAll(self) -> None: ...
    @overload
    def drawAll() -> void: ...
    @overload
    def drawContents(self) -> None: ...
    @overload
    def drawContents() -> void: ...
    @overload
    def drawWidgets(self) -> None: ...
    @overload
    def drawWidgets() -> void: ...
    @overload
    def dropEvent(self) -> bool: ...
    @overload
    def dropEvent(constclassstd) -> bool: ...
    def glfwWindow(self) -> mrpt.pymrpt.GLFWwindow: ...
    def initialize(self, window: mrpt.pymrpt.GLFWwindow, shutdownGLFWOnDestruct: bool) -> None: ...
    def keyCallbackEvent(self, key: int, scancode: int, action: int, mods: int) -> bool: ...
    @overload
    def keyboardCharacterEvent(self, codepoint: int) -> bool: ...
    @overload
    def keyboardCharacterEvent(unsignedint) -> bool: ...
    def keyboardEvent(self, key: int, scancode: int, action: int, modifiers: int) -> bool: ...
    def mouseButtonCallbackEvent(self, button: int, action: int, modifiers: int) -> bool: ...
    @overload
    def mouseModifiers(self) -> int: ...
    @overload
    def mouseModifiers() -> int: ...
    @overload
    def mouseState(self) -> int: ...
    @overload
    def mouseState() -> int: ...
    @overload
    def moveWindowToFront(self, window) -> None: ...
    @overload
    def moveWindowToFront(classnanogui) -> void: ...
    def nvgContext(self) -> mrpt.pymrpt.NVGcontext: ...
    @overload
    def onIdleLoopTasks(self) -> None: ...
    @overload
    def onIdleLoopTasks() -> void: ...
    @overload
    def performLayout(self, ctx: mrpt.pymrpt.NVGcontext) -> None: ...
    @overload
    def performLayout(self) -> None: ...
    @overload
    def performLayout() -> void: ...
    @overload
    def pixelRatio(self) -> float: ...
    @overload
    def pixelRatio() -> float: ...
    def resizeCallback(self, *args, **kwargs) -> Any: ...
    def resizeCallbackEvent(self, width: int, height: int) -> bool: ...
    def scrollCallbackEvent(self, x: float, y: float) -> bool: ...
    @overload
    def setBackground(self, background: Color) -> None: ...
    @overload
    def setBackground(constclassnanogui) -> void: ...
    @overload
    def setCaption(self, caption: str) -> None: ...
    @overload
    def setCaption(conststd) -> void: ...
    @overload
    def setResizeCallback(self, callback) -> None: ...
    @overload
    def setResizeCallback(constclassstd) -> void: ...
    @overload
    def setShutdownGLFWOnDestruct(self, v: bool) -> None: ...
    @overload
    def setShutdownGLFWOnDestruct(bool) -> void: ...
    @overload
    def setVisible(self, visible: bool) -> None: ...
    @overload
    def setVisible(bool) -> void: ...
    @overload
    def shutdownGLFWOnDestruct(self) -> bool: ...
    @overload
    def shutdownGLFWOnDestruct() -> bool: ...

class Window:
    @overload
    def __init__(self, arg0: Window) -> None: ...
    @overload
    def __init__(self, arg0: Window) -> None: ...
    @overload
    def center(self) -> None: ...
    @overload
    def center() -> void: ...
    @overload
    def dispose(self) -> None: ...
    @overload
    def dispose() -> void: ...
    def draw(self, ctx: mrpt.pymrpt.NVGcontext) -> None: ...
    @overload
    def modal(self) -> bool: ...
    @overload
    def modal() -> bool: ...
    def performLayout(self, ctx: mrpt.pymrpt.NVGcontext) -> None: ...
    @overload
    def setModal(self, modal: bool) -> None: ...
    @overload
    def setModal(bool) -> void: ...
    @overload
    def setTitle(self, title: str) -> None: ...
    @overload
    def setTitle(conststd) -> void: ...
    def title(self) -> str: ...
