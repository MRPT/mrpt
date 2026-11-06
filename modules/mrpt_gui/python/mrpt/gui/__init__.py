"""
mrpt.gui — GUI windows for 3D visualization.

Provides:
  - CDisplayWindowGUI: 3D scene viewer window based on nanogui (GLFW), available
    if HAS_NANOGUI (always, in the pip wheel).
  - CDisplayWindow3D : 3D scene viewer window based on wxWidgets, available if
    HAS_WXWIDGETS (not in the pip wheel).
  - CBaseGUIWindow   : Base class of the wxWidgets windows.

Usage example::

    import time
    import mrpt.gui as gui
    import mrpt.viz as viz

    win = gui.CDisplayWindowGUI("My 3D Window", 800, 600)
    scene = viz.Scene()
    scene.insert(viz.stock_objects.CornerXYZ())
    win.background_scene = scene
    while win.spinOnce():  # False once the window is closed
        time.sleep(0.02)
"""

import mrpt.img  # noqa: F401  (CImage in setImageView())
import mrpt.viz  # noqa: F401  (Scene returned by get3DSceneAndLock())

from mrpt.gui._bindings import (
    HAS_NANOGUI,
    HAS_WXWIDGETS,
    CBaseGUIWindow,
    CDisplayWindow3D,
)

__all__ = [
    "HAS_NANOGUI",
    "HAS_WXWIDGETS",
    "CBaseGUIWindow",
    "CDisplayWindow3D",
]

if HAS_NANOGUI:
    from mrpt.gui._bindings import (
        CDisplayWindowGUI,
        CDisplayWindowGUI_Params,
        nanogui_leave,
        nanogui_mainloop,
    )

    __all__ += ["CDisplayWindowGUI", "CDisplayWindowGUI_Params", "nanogui_leave", "nanogui_mainloop"]
