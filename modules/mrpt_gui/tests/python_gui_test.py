#!/usr/bin/env python3
"""
Unit test for the mrpt.gui Python bindings.

Run via CTest (mrpt_add_python_binding_test) or directly:
  python3 modules/mrpt_gui/tests/python_gui_test.py

A window is only opened if there is a display (DISPLAY or WAYLAND_DISPLAY).
"""
import os
import sys

try:
    import mrpt.gui as gui
    import mrpt.viz as viz
except ImportError as e:
    msg = str(e)
    if "_bindings" in msg and "No module named" in msg:
        print(f"SKIP: mrpt.gui bindings not built ({e})", file=sys.stderr)
        sys.exit(0)
    print(f"FAIL: mrpt.gui import error: {e}", file=sys.stderr)
    sys.exit(1)

PASS = 0
FAIL = 0


def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        print(f"  PASS  {name}")
        PASS += 1
    else:
        print(f"  FAIL  {name}" + (f": {detail}" if detail else ""))
        FAIL += 1


print("Module contents")
check("HAS_NANOGUI attr", isinstance(gui.HAS_NANOGUI, bool))
check("HAS_WXWIDGETS attr", isinstance(gui.HAS_WXWIDGETS, bool))
check("CDisplayWindowGUI present if HAS_NANOGUI",
      hasattr(gui, "CDisplayWindowGUI") == gui.HAS_NANOGUI)

has_display = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
if gui.HAS_NANOGUI and has_display:
    print("CDisplayWindowGUI")
    win = gui.CDisplayWindowGUI("python_gui_test", 320, 240)
    check("isOpen after creation", win.isOpen())

    scene = viz.Scene()
    scene.insert(viz.stock_objects.CornerXYZ(1.0))
    win.background_scene = scene
    check("background_scene set", win.background_scene is not None)

    cam = win.camera()
    cam.setAzimuthDegrees(30.0)
    cam.setZoomDistance(5.0)
    check("camera azimuth", abs(cam.azimuth_deg - 30.0) < 1e-4)

    calls = []
    win.addLoopCallback(lambda: calls.append(1))
    ok = all(win.spinOnce() for _ in range(5))
    check("spinOnce returns True while open", ok)
    check("loop callbacks called", len(calls) == 5, f"got {len(calls)}")

    win.setVisible(False)
    check("spinOnce returns False once closed", not win.spinOnce())
else:
    print("SKIP: no display, or MRPT built without nanogui: no window opened")

print(f"\nResults: {PASS} passed, {FAIL} failed")
if FAIL:
    sys.exit(1)
