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

obj_set = CSetOfObjects.Create()
check("CSetOfObjects created", obj_set is not None)

print("Geometry objects")
box = CBox.Create()
check("CBox created", box is not None)

sphere = CSphere.Create(1.0)
check("CSphere created", sphere is not None)

axis = CAxis.Create(-5, -5, -5, 5, 5, 5)
check("CAxis created", axis is not None)

txt = CText.Create("hello")
check("CText created", txt is not None)

lines = CSetOfLines.Create()
lines.appendLine(0, 0, 0, 1, 1, 1)
check("CSetOfLines appendLine", True)

ellip = CEllipsoid3D.Create()
check("CEllipsoid3D created", ellip is not None)

print("CPointCloudColoured")
pc = CPointCloudColoured.Create()
pc.insertPoint({"x": 1.0, "y": 2.0, "z": 3.0, "R": 1.0, "G": 0.0, "B": 0.0, "A": 1.0})
check("insertPoint", pc.size() == 1, f"size={pc.size()}")

print("stock_objects")
corner = stock_objects.CornerXYZ(1.0)
check("CornerXYZ", corner is not None)

print(f"\nResults: {PASS} passed, {FAIL} failed")
sys.exit(1 if FAIL else 0)
