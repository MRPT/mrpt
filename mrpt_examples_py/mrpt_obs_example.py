#!/usr/bin/env python3
"""
mrpt_obs_example.py — sensor observations with mrpt.obs.

Demonstrates:
  - CObservation2DRangeScan: 2D laser scan, numpy helpers
  - CObservationOdometry: odometry reading
  - CObservationIMU: IMU data
  - CActionRobotMovement2D + CActionCollection
  - CSensoryFrame: bundle of observations
"""

import math, numpy as np
from mrpt.obs import (
    CObservation2DRangeScan,
    CObservationOdometry,
    CObservationIMU,
    CActionRobotMovement2D,
    CActionCollection,
    CSensoryFrame,
)
from mrpt.poses import CPose2D, CPose3D
from mrpt.core import Clock

# ---------------------------------------------------------------------------
# CObservation2DRangeScan
# ---------------------------------------------------------------------------
scan = CObservation2DRangeScan()
scan.sensorLabel = "LIDAR_FRONT"
scan.aperture   = math.pi           # 180° FOV
scan.maxRange   = 80.0
scan.rightToLeft = True
scan.resizeScan(360)                # 0.5° resolution

# Fill synthetic ranges (arc at 5 m)
for i in range(360):
    scan.setScanRange(i, 5.0)
    scan.setScanRangeValidity(i, True)

print(f"CObservation2DRangeScan: {scan}")
print(f"  getScanSize = {scan.getScanSize()}")
print(f"  getScanRange(180) = {scan.getScanRange(180):.2f} m")

ranges = scan.getScanRangesAsNumpy()
valid  = scan.getValidRangesAsNumpy()
print(f"  ranges numpy: shape={ranges.shape}, mean={ranges.mean():.2f}")
assert valid.all(), "All rays should be valid"
print("  validity check ✓")

# ---------------------------------------------------------------------------
# CObservationOdometry
# ---------------------------------------------------------------------------
odo = CObservationOdometry()
odo.sensorLabel = "ODO"
odo.odometry    = CPose2D(1.5, 0.0, 0.1)
print(f"\nCObservationOdometry: {odo.odometry}")

# ---------------------------------------------------------------------------
# CActionRobotMovement2D + CActionCollection
# ---------------------------------------------------------------------------
action = CActionRobotMovement2D()
# Probabilistic odometry increment, from a Gaussian motion model:
motion_model = CActionRobotMovement2D.TMotionModelOptions()
motion_model.modelSelection = CActionRobotMovement2D.mmGaussian
action.computeFromOdometry(CPose2D(0.5, 0.0, 0.05), motion_model)
print(f"\nOdometry pose change PDF mean: {action.poseChange.getMean()}")

col = CActionCollection()
col.insert(action)
print(f"\nCActionCollection: {col.size()} actions")
assert col.size() == 1

# ---------------------------------------------------------------------------
# CSensoryFrame — bundle of observations
# ---------------------------------------------------------------------------
sf = CSensoryFrame()
sf.insert(scan)
sf.insert(odo)
print(f"\nCSensoryFrame: {sf.size()} observations")
assert sf.size() == 2

# ---------------------------------------------------------------------------
# Datasets: CRawlog (actions + observations) and CSimpleMap (keyframes)
# ---------------------------------------------------------------------------
import os, tempfile
from mrpt.obs import CRawlog, CSimpleMap
from mrpt.poses import CPose3DPDFGaussian

rawlog = CRawlog()
rawlog.insert(col)  # the action collection
rawlog.insert(sf)   # the sensory frame
simplemap = CSimpleMap()
simplemap.insert(CPose3DPDFGaussian(CPose3D()), sf)

with tempfile.TemporaryDirectory() as tmpdir:
    fname = os.path.join(tmpdir, "demo.rawlog")
    assert rawlog.saveToRawLogFile(fname)
    loaded = CRawlog()
    assert loaded.loadFromRawLogFile(fname)
    print(f"\nRawlog saved and loaded back: {loaded}")
    for i, entry in enumerate(loaded):
        print(f"  [{i}] {type(entry).__name__}")
    # Large rawlogs are better processed as a stream, see
    # CRawlog.ReadFromArchive() in global_localization.py

print(f"CSimpleMap: {simplemap}, first keyframe pose: {simplemap[0].pose.getMean()}")
