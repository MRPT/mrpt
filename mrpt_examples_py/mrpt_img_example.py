#!/usr/bin/env python3
"""
mrpt_img_example.py — image handling with mrpt.img.

Demonstrates:
  - CImage: create from numpy, draw shapes, convert back to numpy
  - TCamera: intrinsic parameter matrix
  - TStereoCamera: stereo rig parameters
  - TColor / TPixelCoord helper types

Optional OpenCV display if cv2 is available.
"""

import numpy as np
from mrpt.img import CImage, TCamera, TStereoCamera, TColor, TPixelCoord, TPixelCoordf

# ---------------------------------------------------------------------------
# TColor and TPixelCoord helper types
# ---------------------------------------------------------------------------
red   = TColor(255, 0, 0)
green = TColor(0, 255, 0)
blue  = TColor(0, 0, 255, 128)   # semi-transparent
print(f"TColor red:   R={red.R} G={red.G} B={red.B} A={red.A}")
print(f"TColor blue (alpha=128): A={blue.A}")

coord = TPixelCoord(320, 240)
print(f"TPixelCoord: x={coord.x} y={coord.y}")

# ---------------------------------------------------------------------------
# CImage — create from a NumPy array (HxWxC uint8)
# ---------------------------------------------------------------------------
np_img = np.zeros((480, 640, 3), dtype=np.uint8)
np_img[100:200, 100:200] = [64, 128, 192]    # fill a rectangle

mrpt_img = CImage(np_img)
print(f"\nCImage: {mrpt_img.getWidth()}x{mrpt_img.getHeight()}, color={mrpt_img.isColor()}")

# Draw a circle and overlay text
mrpt_img.drawCircle(TPixelCoord(320, 240), 50, red, 3)
mrpt_img.textOut(TPixelCoord(10, 10), "MRPT Python", green)

# Convert back to numpy
back = np.array(mrpt_img)
print(f"  back to numpy: shape={back.shape}, dtype={back.dtype}")
assert back.shape == (480, 640, 3)
assert back.dtype == np.uint8
print("  numpy round-trip ✓")

# Optional OpenCV display
try:
    import cv2
    cv2.imshow("mrpt_img_example", back)
    cv2.waitKey(1500)
    cv2.destroyAllWindows()
    print("  (displayed via cv2)")
except ImportError:
    print("  (cv2 not available — skipping display)")

# ---------------------------------------------------------------------------
# CImage.from_numpy — static constructor
# ---------------------------------------------------------------------------
img2 = CImage.from_numpy(np_img)
print(f"\nCImage.from_numpy: {img2.getWidth()}x{img2.getHeight()}")

# ---------------------------------------------------------------------------
# TCamera — monocular intrinsic calibration
# ---------------------------------------------------------------------------
cam = TCamera()
cam.ncols, cam.nrows = 640, 480
cam.fx(525.0)
cam.fy(525.0)
cam.cx(319.5)
cam.cy(239.5)
cam.distortion = [0.1, -0.05, 0.0, 0.0, 0.0]   # k1 k2 p1 p2 k3

K = cam.intrinsicParams   # 3x3 numpy array
print(f"\nTCamera intrinsic matrix K:\n{K}")
assert K.shape == (3, 3)
assert abs(K[0, 0] - 525.0) < 1e-6
print("  intrinsic matrix ✓")

# ---------------------------------------------------------------------------
# TStereoCamera — stereo rig
# ---------------------------------------------------------------------------
sc = TStereoCamera()
sc.leftCamera  = cam
sc.rightCamera = cam
print(f"\nTStereoCamera:\n{sc}")
