#!/usr/bin/env python3
"""Smoke tests for mrpt.slam Python bindings."""
import sys, math

try:
    from mrpt.slam import CICP, CMetricMapBuilderICP, TICPAlgorithm
except ImportError as e:
    msg = str(e)
    if "_bindings" in msg and "No module named" in msg:
        print(f"SKIP: mrpt.slam bindings not built ({e})", file=sys.stderr)
        sys.exit(0)
    print(f"FAIL: mrpt.slam import error: {e}", file=sys.stderr)
    sys.exit(1)

PASS = FAIL = 0

def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        print(f"  PASS  {name}"); PASS += 1
    else:
        print(f"  FAIL  {name}" + (f": {detail}" if detail else "")); FAIL += 1

print("CICP")
icp = CICP()
check("CICP created", icp is not None)
check("default algo exists", hasattr(icp.options, "ICP_algorithm"))
icp.options.ICP_algorithm = TICPAlgorithm.icpClassic
check("algo setter", icp.options.ICP_algorithm == TICPAlgorithm.icpClassic)

print("CMetricMapBuilderICP")
builder = CMetricMapBuilderICP()
builder.ICP_options.insertionLinDistance = 0.5
builder.ICP_options.insertionAngDistance = math.radians(30)
builder.initialize()
check("initialize ok", True)

pose_pdf = builder.getCurrentPoseEstimation()
check("getCurrentPoseEstimation not None", pose_pdf is not None)

import math as _m
import numpy as _np
from mrpt.slam import (
    CMonteCarloLocalization2D, TMonteCarloLocalizationParams, TKLDParams, CMetricMapBuilderRBPF,
    CICPOptions,
)
from mrpt.bayes import CParticleFilter, TParticleFilterAlgorithm, TParticleFilterStats
from mrpt.maps import COccupancyGridMap2D as _Grid, TSetOfMetricMapInitializers
from mrpt.obs import (
    CObservation2DRangeScan as _Scan, CSensoryFrame as _SF, CActionCollection as _AC,
    CActionRobotMovement2D as _ARM,
)
from mrpt.poses import CPose2D as _P2, CPose3D as _P3
from mrpt.math import TPose2D
from mrpt.config import CConfigFileMemory


def room_scan(robot_x, robot_y, half=3.0, n=361):
    """Simulated 360 deg scan inside a square room [-half, half]^2."""
    sc = _Scan()
    sc.aperture = 2 * _m.pi
    sc.maxRange = 20.0
    sc.resizeScan(n)
    for i in range(n):
        ang = -_m.pi + 2 * _m.pi * i / (n - 1)
        c, s_ = _m.cos(ang), _m.sin(ang)
        tx = ((half if c > 0 else -half) - robot_x) / c if abs(c) > 1e-9 else 1e9
        ty = ((half if s_ > 0 else -half) - robot_y) / s_ if abs(s_) > 1e-9 else 1e9
        sc.setScanRange(i, min(tx, ty))
        sc.setScanRangeValidity(i, True)
    return sc


print("Monte Carlo localization")
grid = _Grid(-4.0, 4.0, -4.0, 4.0, 0.05)
for x, y in [(0.0, 0.0), (1.0, 1.0), (-1.0, 0.5)]:
    grid.insertObservation(room_scan(x, y), _P3(x, y, 0.0, 0.0, 0.0, 0.0))

mcl = CMonteCarloLocalization2D(1)
mcl.options.metricMap = grid
check("options.metricMap", mcl.options.metricMap is not None)
mcl.resetAroundSetOfPoses([TPose2D(0.8, -0.5, 0.0)], 300, 0.6, 0.6, 0.2)
pf = CParticleFilter()
pf.options.PF_algorithm = TParticleFilterAlgorithm.StandardProposal
true_x, true_y = 1.0, -0.8
for step in range(5):
    act = _AC()
    odo = _ARM()
    opts = _ARM.TMotionModelOptions()
    opts.gaussianModel.minStdXY = 0.05
    odo.computeFromOdometry(_P2(0.0, 0.0, 0.0), opts)
    act.insert(odo)
    sf = _SF()
    sf.insert(room_scan(true_x, true_y))
    stats = pf.executeOn(mcl, act, sf)
check("executeOn returns stats", isinstance(stats, TParticleFilterStats))
est = mcl.getMean()
err = _m.hypot(est.x - true_x, est.y - true_y)
check("MCL converges", err < 0.25, f"estimate {est}, error {err:.3f} m")
check("MCL visualization", len(mcl.getVisualization()) > 0)
kld = TKLDParams()
kld.loadFromConfigFile(CConfigFileMemory("[KLD]\nKLD_maxSampleSize=500\n"), "KLD")
check("TKLDParams from config", kld.KLD_maxSampleSize == 500)
mcl.options.KLD_params = kld
check("KLD_params assignment", mcl.options.KLD_params.KLD_maxSampleSize == 500)

print("CICPOptions / CMetricMapBuilderICPOptions config loading")
icp_opts = CICPOptions()
icp_opts.loadFromConfigFile(CConfigFileMemory("[ICP]\nmaxIterations=77\n"), "ICP")
check("CICPOptions.loadFromConfigFile", icp_opts.maxIterations == 77)

print("CMetricMapBuilderRBPF")
ro = CMetricMapBuilderRBPF.TConstructionOptions()
ro.mapsInitializers.loadFromConfigFile(CConfigFileMemory(
    "[M]\noccupancyGrid_count=1\n[M_occupancyGrid_00_creationOpts]\n"
    "min_x=-5\nmax_x=5\nmin_y=-5\nmax_y=5\nresolution=0.1\n"), "M")
ro.PF_options.sampleSize = 5
ro.PF_options.PF_algorithm = TParticleFilterAlgorithm.OptimalProposal
ro.insertionLinDistance = 0.1
rbpf = CMetricMapBuilderRBPF(ro)
rbpf.initialize()
for i in range(4):
    act = _AC()
    odo = _ARM()
    odo.computeFromOdometry(_P2(0.2 if i else 0.0, 0.0, 0.0), _ARM.TMotionModelOptions())
    act.insert(odo)
    sf = _SF()
    sf.insert(room_scan(0.2 * i, 0.0))
    rbpf.processActionObservation(act, sf)
check("RBPF built map", rbpf.getCurrentlyBuiltMapSize() >= 1,
      f"got {rbpf.getCurrentlyBuiltMapSize()}")
path = rbpf.getCurrentMostLikelyPath()
check("RBPF path", len(path) >= 1)
check("RBPF metric map", len(rbpf.getCurrentlyBuiltMetricMap()) == 1)
check("RBPF pose estimate", rbpf.getCurrentPoseEstimation() is not None)

print(f"\nResults: {PASS} passed, {FAIL} failed")
sys.exit(1 if FAIL else 0)
