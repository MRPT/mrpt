#!/usr/bin/env python3
"""Smoke tests for mrpt.obs Python bindings."""
import sys, math

try:
    from mrpt.obs import (
        CObservation2DRangeScan, CObservationOdometry,
        CObservationIMU, CActionCollection,
        CActionRobotMovement2D, CSensoryFrame,
    )
except ImportError as e:
    msg = str(e)
    if "_bindings" in msg and "No module named" in msg:
        print(f"SKIP: mrpt.obs bindings not built ({e})", file=sys.stderr)
        sys.exit(0)
    print(f"FAIL: mrpt.obs import error: {e}", file=sys.stderr)
    sys.exit(1)

import numpy as np

PASS = FAIL = 0

def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        print(f"  PASS  {name}"); PASS += 1
    else:
        print(f"  FAIL  {name}" + (f": {detail}" if detail else "")); FAIL += 1

print("CObservation2DRangeScan")
scan = CObservation2DRangeScan()
scan.aperture = math.pi
scan.maxRange = 80.0
N = 100
scan.resizeScan(N)
for i in range(N):
    scan.setScanRange(i, 1.0 + 4.0 * i / (N - 1))
    scan.setScanRangeValidity(i, True)
check("aperture",  abs(scan.aperture - math.pi) < 1e-6)
check("maxRange",  abs(scan.maxRange - 80.0) < 1e-9)
check("getScanSize", scan.getScanSize() == N)
back = scan.getScanRangesAsNumpy()
check("ranges shape",  back.shape == (N,), f"got {back.shape}")
check("ranges values", abs(float(back[0]) - 1.0) < 1e-4 and abs(float(back[-1]) - 5.0) < 1e-4)

print("CObservationOdometry")
odo = CObservationOdometry()
check("odometry created", odo is not None)

print("CSensoryFrame")
sf = CSensoryFrame()
sf.insert(scan)
check("SF size == 1", sf.size() == 1)

print("CActionCollection")
ac = CActionCollection()
check("action collection empty", ac.size() == 0)

import os, tempfile
from mrpt.obs import (
    CObservation3DRangeScan, T3DPointsProjectionParams, CObservationGPS, GnssFixType, gnss,
    CActionRobotMovement3D, CRawlog, CSimpleMap, TSetOfMetricMapInitializers,
)
from mrpt.poses import CPose2D, CPose3D, CPose3DPDFGaussian, CPosePDF

print("CObservation3DRangeScan")
o3d = CObservation3DRangeScan()
W, H = 8, 6
o3d.cameraParams.ncols = W
o3d.cameraParams.nrows = H
o3d.cameraParams.fx = 5.0
o3d.cameraParams.fy = 5.0
o3d.cameraParams.cx = (W - 1) / 2
o3d.cameraParams.cy = (H - 1) / 2
o3d.setRangeImageFromNumpy(np.full((H, W), 2.0, dtype=np.float32))
check("hasRangeImage", o3d.hasRangeImage)
ri = o3d.getRangeImageAsNumpy()
check("range image shape", ri.shape == (H, W), f"got {ri.shape}")
check("range image meters", np.allclose(ri, 2.0, atol=o3d.rangeUnits))
check("raw range image", o3d.getRangeImageRawAsNumpy().dtype == np.uint16)
o_nan = CObservation3DRangeScan()
bad = np.full((2, 3), 1.5, dtype=np.float32)
bad[0, 0] = np.nan
bad[1, 2] = np.inf
o_nan.setRangeImageFromNumpy(bad)
raw_nan = o_nan.getRangeImageRawAsNumpy()
check("NaN/inf ranges stored as 0", raw_nan[0, 0] == 0 and raw_nan[1, 2] == 0 and raw_nan[0, 1] > 0)
o3d.unprojectInto()
pts = o3d.getPoints3DAsNumpy()
check("unprojected point count", pts.shape == (W * H, 3), f"got {pts.shape}")
# MRPT camera frames look along +X: every point is at depth 2 m.
check("unprojected depth", np.allclose(pts[:, 0], 2.0, atol=1e-3), f"got {pts[:3]}")

print("CObservationGPS")
gps = CObservationGPS()
check("no GGA initially", not gps.hasGGA() and gps.getGGA() is None)
gga = gnss.Message_NMEA_GGA()
gga.fields.latitude_degrees = 36.8
gga.fields.longitude_degrees = -2.4
gga.fields.fix_quality = 4
gga.fields.satellitesUsed = 11
gps.setGGA(gga)
gps.fix_type = GnssFixType.RTK_FIXED
back_gga = gps.getGGA()
check("GGA round-trip", gps.hasGGA() and abs(back_gga.fields.latitude_degrees - 36.8) < 1e-12
      and back_gga.fields.satellitesUsed == 11)
check("fix type", gps.fix_type == GnssFixType.RTK_FIXED)
check("no ENU covariance by default", gps.covariance_enu is None)

print("CActionRobotMovement2D.computeFromOdometry")
act2d = CActionRobotMovement2D()
opts = CActionRobotMovement2D.TMotionModelOptions()
opts.modelSelection = CActionRobotMovement2D.mmGaussian
act2d.computeFromOdometry(CPose2D(1.0, 0.5, 0.1), opts)
pc = act2d.poseChange
check("poseChange is a CPosePDF", isinstance(pc, CPosePDF), f"got {type(pc)}")
check("poseChange mean", abs(pc.getMean().x - 1.0) < 1e-9 and abs(pc.getMean().y - 0.5) < 1e-9)
check("odometry stored", abs(act2d.rawOdometryIncrementReading.x - 1.0) < 1e-12)
sample = act2d.drawSingleSample()
check("motion model sample", abs(sample.x - 1.0) < 1.0)

print("CActionRobotMovement3D.computeFromOdometry")
act3d = CActionRobotMovement3D()
act3d.computeFromOdometry(CPose3D(1.5, 0.0, 0.0, 0.0, 0.0, 0.0),
                          CActionRobotMovement3D.TMotionModelOptions())
check("3D poseChange mean", abs(act3d.poseChange.mean.x - 1.5) < 0.01,
      f"got {act3d.poseChange.mean}")

print("CActionCollection.getBestMovementEstimation")
acts = CActionCollection()
check("no movement -> None", acts.getBestMovementEstimation() is None)
acts.insert(act2d)
best = acts.getBestMovementEstimation()
check("best movement", best is not None and abs(best.rawOdometryIncrementReading.x - 1.0) < 1e-12)

print("CRawlog")
rawlog = CRawlog()
for i in range(3):
    ac = CActionCollection()
    a = CActionRobotMovement2D()
    a.computeFromOdometry(CPose2D(0.1 * (i + 1), 0.0, 0.0), opts)
    ac.insert(a)
    frame = CSensoryFrame()
    frame.insert(scan)
    rawlog.insert(ac)
    rawlog.insert(frame)
check("rawlog size", len(rawlog) == 6, f"got {len(rawlog)}")
check("entry types", rawlog.getType(0) == CRawlog.etActionCollection
      and rawlog.getType(1) == CRawlog.etSensoryFrame)
check("getAsObservations", len(rawlog.getAsObservations(1)) == 1)
check("iteration", sum(1 for _ in rawlog) == 6)
with tempfile.TemporaryDirectory() as tmpdir:
    fname = os.path.join(tmpdir, "test.rawlog")
    check("saveToRawLogFile", rawlog.saveToRawLogFile(fname))
    loaded = CRawlog()
    check("loadFromRawLogFile", loaded.loadFromRawLogFile(fname) and len(loaded) == 6)
    check("loaded entries keep their class",
          isinstance(loaded[1], CSensoryFrame) and isinstance(loaded[0], CActionCollection))

    # Streamed reading, as used to process large datasets:
    from mrpt.io import CCompressedInputStream
    from mrpt.serialization import archiveFrom
    f = CCompressedInputStream(fname)
    arch = archiveFrom(f)
    entry = 0
    n_pairs = 0
    check_ok = False
    while True:
        ok, entry, ac_read, sf_read, obs_read = CRawlog.ReadFromArchive(arch, entry)
        if not ok:
            break
        n_pairs += 1
        check_ok = ac_read is not None and sf_read is not None and obs_read is None
    check("ReadFromArchive pairs", n_pairs == 3, f"got {n_pairs}")
    check("ReadFromArchive types", check_ok)

    # EOF on generic object reading maps to EOFError:
    f2 = CCompressedInputStream(fname)
    arch2 = archiveFrom(f2)
    n_objs = 0
    try:
        while True:
            arch2.ReadObject()
            n_objs += 1
    except EOFError:
        pass
    check("ReadObject until EOFError", n_objs == 6, f"got {n_objs}")

print("CSimpleMap")
smap = CSimpleMap()
for i in range(4):
    frame = CSensoryFrame()
    frame.insert(scan)
    smap.insert(CPose3DPDFGaussian(CPose3D(float(i), 0.0, 0.0, 0.0, 0.0, 0.0)), frame)
check("simplemap size", len(smap) == 4)
kf = smap[2]
check("keyframe pose", abs(kf.pose.getMean().x - 2.0) < 1e-12)
check("keyframe sf", len(kf.sf) == 1)
check("keyframe iteration", [round(k.pose.getMean().x) for k in smap] == [0, 1, 2, 3])
with tempfile.TemporaryDirectory() as tmpdir:
    fname = os.path.join(tmpdir, "test.simplemap")
    check("simplemap save", smap.saveToFile(fname))
    smap2 = CSimpleMap()
    check("simplemap load", smap2.loadFromFile(fname) and len(smap2) == 4)
smap.changeCoordinatesOrigin(CPose3D(10.0, 0.0, 0.0, 0.0, 0.0, 0.0))
check("changeCoordinatesOrigin", abs(smap[0].pose.getMean().x - 10.0) < 1e-9)

print("TSetOfMetricMapInitializers (empty)")
check("empty map set", len(TSetOfMetricMapInitializers()) == 0)

print(f"\nResults: {PASS} passed, {FAIL} failed")
sys.exit(1 if FAIL else 0)
