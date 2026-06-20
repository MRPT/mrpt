"""
mrpt.gui — GUI windows for 3D visualization.

Provides:
  - CBaseGUIWindow  : Abstract base for all MRPT GUI windows
  - CDisplayWindow3D: Interactive 3D OpenGL scene viewer window

Usage example::

    import mrpt.gui as gui
    import mrpt.viz as viz

    win = gui.CDisplayWindow3D("My 3D Window", 800, 600)
    scene = win.get3DSceneAndLock()
    scene.insert(viz.stock_objects.CornerXYZ())
    win.unlockAccess3DScene()
    win.forceRepaint()
    win.waitForKey()
"""

from mrpt.gui._bindings import (
    CBaseGUIWindow,
    CDisplayWindow3D,
)

__all__ = [
    "CBaseGUIWindow",
    "CDisplayWindow3D",
]
