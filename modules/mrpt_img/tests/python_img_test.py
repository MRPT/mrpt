#!/usr/bin/env python3
"""Smoke tests for mrpt.img Python bindings."""
import sys

try:
    from mrpt.img import CImage, TColor, TCamera, TPixelCoord
except ImportError as e:
    msg = str(e)
    if "_bindings" in msg and "No module named" in msg:
        print(f"SKIP: mrpt.img bindings not built ({e})", file=sys.stderr)
        sys.exit(0)
    print(f"FAIL: mrpt.img import error: {e}", file=sys.stderr)
    sys.exit(1)

import numpy as np

PASS = FAIL = 0

def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        print(f"  PASS  {name}"); PASS += 1
    else:
        print(f"  FAIL  {name}" + (f": {detail}" if detail else "")); FAIL += 1

print("CImage")
img = CImage()
check("default CImage created", img is not None)

# from_numpy round-trip
arr = np.zeros((100, 200, 3), dtype=np.uint8)
arr[50, 100, :] = [128, 64, 32]
img2 = CImage.from_numpy(arr)
check("from_numpy width",  img2.getWidth()  == 200)
check("from_numpy height", img2.getHeight() == 100)

back = img2.as_numpy()
check("as_numpy shape", back.shape == (100, 200, 3), f"got {back.shape}")
check("pixel value", list(back[50, 100]) == [128, 64, 32], f"got {list(back[50,100])}")

print("TColor")
c = TColor(255, 128, 0, 255)
check("R", c.R == 255)
check("G", c.G == 128)
check("B", c.B == 0)

print("TCamera")
cam = TCamera()
check("TCamera ncols accessible", cam.ncols >= 0)  # just check it's accessible

print("TPixelCoord")
px = TPixelCoord(10, 20)
check("x", px.x == 10)
check("y", px.y == 20)

print(f"\nResults: {PASS} passed, {FAIL} failed")
sys.exit(1 if FAIL else 0)
