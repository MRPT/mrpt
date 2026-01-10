#!/bin/env python3

import mrpt.img
import numpy as np
import cv2

# 1. Create an MRPT Image from a NumPy array (e.g., from OpenCV)
cv_img = np.zeros((480, 640, 3), dtype=np.uint8)
mrpt_img = mrpt.img.CImage(cv_img)

# 2. Use MRPT's drawing functions
mrpt_img.drawCircle(mrpt.img.TPixelCoord(320, 240), 50, mrpt.img.Color.RED, 3)
mrpt_img.textOut(mrpt.img.TPixelCoord(10, 10),
                 "MRPT + NumPy", mrpt.img.Color.GREEN)

# 3. Convert back to NumPy/OpenCV (Zero-Copy)
# Because of __array__, cv2 functions accept mrpt_img directly!
final_view = np.array(mrpt_img)
cv2.imshow("Result", final_view)
cv2.waitKey(0)

# 4. Handle Camera Calibration
cam = mrpt.img.TCamera()
cam.ncols, cam.nrows = 640, 480
cam.dist = [0.1, -0.05, 0, 0, 0]  # k1, k2, p1, p2, k3
K = cam.intrinsicParams()  # Returns a 3x3 NumPy array
print(f"Intrinsic Matrix:\n{K}")
