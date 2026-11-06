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
from __future__ import annotations
import mrpt as mrpt
from mrpt.gui._bindings import CBaseGUIWindow as CBaseGUIWindow
from mrpt.gui._bindings import CDisplayWindow3D as CDisplayWindow3D
from mrpt.gui._bindings import CDisplayWindowGUI as CDisplayWindowGUI
from mrpt.gui._bindings import CDisplayWindowGUI_Params as CDisplayWindowGUI_Params
from mrpt.gui._bindings import nanogui_leave as nanogui_leave
from mrpt.gui._bindings import nanogui_mainloop as nanogui_mainloop
from . import _bindings
__all__: list = ['HAS_NANOGUI', 'HAS_WXWIDGETS', 'CBaseGUIWindow', 'CDisplayWindow3D', 'CDisplayWindowGUI', 'CDisplayWindowGUI_Params', 'nanogui_leave', 'nanogui_mainloop']
HAS_NANOGUI: bool
HAS_WXWIDGETS: bool
