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

print("Line width")
_lw_bad = []
for _name in ["CSetOfLines", "CSimpleLine", "CGridPlaneXY", "CGridPlaneXZ", "CAxis", "CBox",
              "CEllipsoid2D", "CEllipsoid3D"]:
    _o = getattr(_viz, _name)()
    _o.setLineWidth(3.5)
    if abs(_o.getLineWidth() - 3.5) > 1e-6:
        _lw_bad.append(_name)
check("setLineWidth()/getLineWidth()", not _lw_bad, f"wrong: {_lw_bad}")

print("Ellipsoid drawing parameters")
from mrpt.math import CMatrixDouble33 as _M33
_ell = CEllipsoid3D()
# pybind11 < 2.12 cannot take NumPy >= 2 arrays (mrpt.rtti warns about it):
import mrpt.rtti._bindings as _rtti
_numpy_to_mrpt_ok = not (int(np.__version__.split(".")[0]) >= 2 and
                         tuple(_rtti._pybind11_version[:2]) < (2, 12))
if _numpy_to_mrpt_ok:
    _ell.setCovMatrix(_M33(np.diag([1.0, 0.5, 0.25])))
_ell.enableDrawSolid3D(True)
_ell.setNumberOfSegments(32)
_ell.setQuantiles(2.0)
check("getNumberOfSegments()", _ell.getNumberOfSegments() == 32)
check("getQuantiles()", abs(_ell.getQuantiles() - 2.0) < 1e-6)

print("CCamera")
from mrpt.img import TCamera as _TCamera
from mrpt.math import TPoint3D as _TP3
_cam = Scene().getViewport("main").getCamera()
_cam.setPointingAt(1.0, 2.0, 3.0)
_p = _cam.getPointingAt()
check("setPointingAt(x, y, z)", (_p.x, _p.y, _p.z) == (1.0, 2.0, 3.0), f"{_p}")
_cam.setPointingAt(_TP3(-1.0, 0.5, 0.0))
check("setPointingAt(TPoint3D)", abs(_cam.getPointingAt().x + 1.0) < 1e-6)
_cam.setAzimuthDegrees(30.0)
_cam.setElevationDegrees(20.0)
_cam.setZoomDistance(7.0)
_cam.setRollDegrees(5.0)
check("orbit getters", (_cam.getAzimuthDegrees(), _cam.getElevationDegrees(),
                        _cam.getZoomDistance(), _cam.getRollDegrees()) == (30.0, 20.0, 7.0, 5.0))
_cam.setOrthogonal()
check("setOrthogonal()", _cam.isOrthogonal() and not _cam.isProjective())
_cam.setProjectiveModel()
_cam.setProjectiveFOVdeg(60.0)
check("setProjectiveFOVdeg()",
      _cam.isProjective() and abs(_cam.getProjectiveFOVdeg() - 60.0) < 1e-6)
check("no pinhole model by default", not _cam.hasPinholeModel() and _cam.getPinholeModel() is None)
_K = _TCamera()
_K.ncols, _K.nrows, _K.fx, _K.fy, _K.cx, _K.cy = 640, 480, 400.0, 410.0, 320.0, 240.0
_cam.setProjectiveFromPinhole(_K)
check("setProjectiveFromPinhole()",
      _cam.hasPinholeModel() and abs(_cam.getPinholeModel().fy - 410.0) < 1e-9)
_cam.clearPinholeModel()
check("clearPinholeModel()", not _cam.hasPinholeModel())
_cam.set6DOFMode(True)
check("set6DOFMode()", _cam.is6DOFMode())

print(f"\nResults: {PASS} passed, {FAIL} failed")
sys.exit(1 if FAIL else 0)
