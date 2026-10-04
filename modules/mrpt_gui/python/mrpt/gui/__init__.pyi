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
from __future__ import annotations
from mrpt.gui._bindings import CBaseGUIWindow as CBaseGUIWindow
from mrpt.gui._bindings import CDisplayWindow3D as CDisplayWindow3D
from . import _bindings
__all__: list = ['CBaseGUIWindow', 'CDisplayWindow3D']
