#!/usr/bin/env python3
"""
mrpt_slam_example.py — ICP and ICP-SLAM with mrpt.slam.

Demonstrates:
  - CICP: align two point-cloud maps, inspect TICPReturnInfo
  - CMetricMapBuilderICP: incremental ICP-SLAM from a sequence of laser scans
"""

import math, numpy as np
from mrpt.slam import (
    CICP, CICPOptions, TICPReturnInfo,
    CMetricMapBuilderICP, CMetricMapBuilderICPOptions,
    TICPAlgorithm, TICPCovarianceMethod,
)
from mrpt.maps import CSimplePointsMap
from mrpt.obs import CObservation2DRangeScan, CActionCollection, CSensoryFrame
from mrpt.poses import CPosePDFGaussian, CPose2D

# ---------------------------------------------------------------------------
# Build two synthetic point clouds offset by 1 m along X
# ---------------------------------------------------------------------------
def make_cloud(cx, cy, n_points=50, radius=3.0):
    pts = CSimplePointsMap()
    for i in range(n_points):
        angle = 2 * math.pi * i / n_points
        pts.insertPoint(cx + radius * math.cos(angle),
                        cy + radius * math.sin(angle), 0.0)
    return pts

cloud1 = make_cloud(0.0, 0.0)
cloud2 = make_cloud(1.0, 0.0)   # shifted 1 m in X

# ---------------------------------------------------------------------------
# CICP — align cloud2 onto cloud1
# ---------------------------------------------------------------------------
opts = CICPOptions()
opts.maxIterations = 40
opts.thresholdDist = 0.5
opts.thresholdAng  = math.radians(5)

icp = CICP(opts)

init_est = CPosePDFGaussian()   # identity initial guess

result_pdf, info = icp.AlignPDF(cloud1, cloud2, init_est)
print(f"ICP result:")
print(f"  nIterations = {info.nIterations}")
print(f"  goodness    = {info.goodness:.4f}")
print(f"  quality     = {info.quality:.4f}")
print(f"  {info}")

# ---------------------------------------------------------------------------
# CMetricMapBuilderICP — simple ICP-SLAM pipeline
# ---------------------------------------------------------------------------
def make_scan(radius=5.0, n_rays=180, noise=0.01):
    """Generate a synthetic 180° laser scan (arc at `radius` metres)."""
    scan = CObservation2DRangeScan()
    scan.aperture    = math.pi
    scan.maxRange    = 20.0
    scan.rightToLeft = True
    scan.resizeScan(n_rays)
    rng = np.random.default_rng(0)
    for i in range(n_rays):
        r = radius + rng.normal(0, noise)
        scan.setScanRange(i, float(r))
        scan.setScanRangeValidity(i, True)
    return scan

builder = CMetricMapBuilderICP()
builder.ICP_options.insertionLinDistance = 0.3
builder.ICP_options.insertionAngDistance = math.radians(10)
builder.useSimplePointsMap()  # configure mapInitializers with a CSimplePointsMap (must be before initialize())
builder.initialize()

# Feed 5 scans; each call to processObservation builds the map
for step in range(5):
    scan = make_scan(radius=4.0 + step * 0.1)
    scan.sensorLabel = "LASER"
    builder.processObservation(scan)

print(f"\nCMetricMapBuilderICP after 5 scans:")
print(f"  map size = {builder.getCurrentlyBuiltMapSize()} keyframes")
pose_pdf = builder.getCurrentPoseEstimation()
print(f"  current pose PDF: {pose_pdf}")
xs, ys = builder.getCurrentMapPoints()
print(f"  point-map size: {len(xs)} points")
