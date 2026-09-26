#!/usr/bin/env python3
"""Smoke tests for mrpt.viz Python bindings (headless — no window opened)."""
import sys

try:
    from mrpt.viz import (
        Scene, CSetOfObjects, CPointCloud, CPointCloudColoured,
        CBox, CSphere, CAxis, CText, CSetOfLines, CEllipsoid3D,
        stock_objects,
    )
except ImportError as e:
    msg = str(e)
    if "_bindings" in msg and "No module named" in msg:
        print(f"SKIP: mrpt.viz bindings not built ({e})", file=sys.stderr)
        sys.exit(0)
    print(f"FAIL: mrpt.viz import error: {e}", file=sys.stderr)
    sys.exit(1)

import numpy as np

PASS = FAIL = 0

def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        print(f"  PASS  {name}"); PASS += 1
    else:
        print(f"  FAIL  {name}" + (f": {detail}" if detail else "")); FAIL += 1

print("Scene / CSetOfObjects")
scene = Scene()
check("Scene created", scene is not None)

obj_set = CSetOfObjects()
check("CSetOfObjects created", obj_set is not None)

print("Geometry objects")
box = CBox()
check("CBox created", box is not None)

sphere = CSphere(1.0)
check("CSphere created", sphere is not None)

axis = CAxis(-5, -5, -5, 5, 5, 5)
check("CAxis created", axis is not None)

txt = CText("hello")
check("CText created", txt is not None)

lines = CSetOfLines()
lines.appendLine(0, 0, 0, 1, 1, 1)
check("CSetOfLines appendLine", True)

ellip = CEllipsoid3D()
check("CEllipsoid3D created", ellip is not None)

print("CPointCloudColoured")
pc = CPointCloudColoured()
pc.push_back(1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 1.0)
check("push_back", pc.size() == 1, f"size={pc.size()}")

print("stock_objects")
corner = stock_objects.CornerXYZ(1.0)
check("CornerXYZ", corner is not None)

print("CVisualObject base access on every class")
# Most classes inherit CVisualObject virtually: base methods must reach the
# right sub-object (they used to crash or corrupt memory).
import inspect as _inspect
import mrpt.viz as _viz
from mrpt.poses import CPose3D as _CP3
from mrpt.img import TColorf as _TColorf
_base = _viz.CSetOfObjects.__mro__[1]
_bad = []
for _name in dir(_viz):
    _cls = getattr(_viz, _name)
    if not (_inspect.isclass(_cls) and issubclass(_cls, _base) and _cls is not _base):
        continue
    try:
        _o = _cls()
    except TypeError:
        continue  # no default constructor
    _o.name = "obj_" + _name
    _o.setColor(_TColorf(0.0, 1.0, 0.0))
    _o.setPose(_CP3(1.0, 2.0, 3.0, 0.0, 0.0, 0.0))
    _o.visible = False
    if not (_o.name == "obj_" + _name and abs(_o.getPose().z - 3.0) < 1e-9
            and abs(_o.getColor().G - 1.0) < 1e-6 and not _o.visible):
        _bad.append(_name)
check("base methods on all CVisualObject classes", not _bad, f"wrong: {_bad}")

print("posePDF2opengl")
from mrpt.poses import CPose3DPDFGaussian as _G3
check("posePDF2opengl", len(_viz.posePDF2opengl(_G3(_CP3()))) > 0)

print(f"\nResults: {PASS} passed, {FAIL} failed")
sys.exit(1 if FAIL else 0)
