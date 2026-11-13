#!/usr/bin/env python3
"""Smoke tests for mrpt.tfest Python bindings."""
import sys, math

try:
    from mrpt.tfest import TMatchingPair, TMatchingPairList, se2_l2
except ImportError as e:
    msg = str(e)
    if "_bindings" in msg and "No module named" in msg:
        print(f"SKIP: mrpt.tfest bindings not built ({e})", file=sys.stderr)
        sys.exit(0)
    print(f"FAIL: mrpt.tfest import error: {e}", file=sys.stderr)
    sys.exit(1)

PASS = FAIL = 0

def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        print(f"  PASS  {name}"); PASS += 1
    else:
        print(f"  FAIL  {name}" + (f": {detail}" if detail else "")); FAIL += 1

print("TMatchingPair / TMatchingPairList")
pair = TMatchingPair()
pair.localIdx = 0
pair.globalIdx = 0
check("pair created", pair is not None)

lst = TMatchingPairList()
lst.push_back(pair)
check("list size == 1", lst.size() == 1)

print("se2_l2")
# se2_l2 requires TMatchingPair with local_pt/global_pt (TPoint3Df),
# which needs mrpt.math TPoint3Df. Just test it's callable.
correspondences = TMatchingPairList()
check("empty correspondences list", correspondences.size() == 0)

# ---------------------------------------------------------------------------
# Estimators, with points transformed by a known pose
# ---------------------------------------------------------------------------
import random
import numpy as np
import mrpt.random
from mrpt.math import TPoint2D, TPoint3D, TPoint3Df
from mrpt.poses import CPose2D, CPose3D, CPosePDFSOG
from mrpt.tfest import (
    TMatchingPair_d, TMatchingPairList_d, TSE2RobustParams, TSE2RobustResult,
    TSE3RobustParams, se2_l2_with_covariance, se2_l2_robust, se3_l2, se3_l2_robust,
)

random.seed(42)
mrpt.random.Randomize(42)


def close(a, b, tol):
    return abs(a - b) < tol


print("TMatchingPair constructors")
pr = TMatchingPair(3, 4, TPoint3Df(1, 2, 3), TPoint3Df(4, 5, 6))
check("ctor from points", pr.globalIdx == 3 and pr.localIdx == 4 and
      close(pr.global_pt.z, 3, 1e-6) and close(pr.local_pt.x, 4, 1e-6))
pr = TMatchingPair(3, 4, 1, 2, 3, 4, 5, 6)
check("ctor from coordinates", close(pr.global_pt.y, 2, 1e-6) and close(pr.local_pt.z, 6, 1e-6))
pr_d = TMatchingPair_d(1, 2, TPoint3D(0.1, 0.2, 0.3), TPoint3D(1, 2, 3))
check("TMatchingPair_d ctor", close(pr_d.global_pt.x, 0.1, 1e-15))

print("se2_l2 / se2_l2_with_covariance")
q2 = CPose2D(1.0, 2.0, 0.3)
corrs2 = TMatchingPairList()
locals2 = []
for i in range(20):
    lp = TPoint2D(random.uniform(-5, 5), random.uniform(-5, 5))
    gp = q2.composePoint(lp)
    locals2.append(lp)
    corrs2.push_back(TMatchingPair(i, i, TPoint3Df(gp.x, gp.y, 0), TPoint3Df(lp.x, lp.y, 0)))
ok, est = se2_l2(corrs2)
check("se2_l2", ok and close(est.x, 1.0, 1e-4) and close(est.y, 2.0, 1e-4) and
      close(est.phi, 0.3, 1e-5), f"got {est}")

ok, est, cov = se2_l2_with_covariance(corrs2)
check("se2_l2_with_covariance pose", ok and close(est.x, 1.0, 1e-4) and close(est.phi, 0.3, 1e-5))
# Reference: the normalized covariance in closed form (see se2_l2() docs).
ga = np.array([[c.global_pt.x, c.global_pt.y] for c in corrs2], dtype=float)
lb = np.array([[c.local_pt.x, c.local_pt.y] for c in corrs2], dtype=float)
N = len(ga)
ma_, mb_ = ga.mean(axis=0), lb.mean(axis=0)
da, db = ga - ma_, lb - mb_
Ax = N * np.sum(da[:, 0] * db[:, 0] + da[:, 1] * db[:, 1])
Ay = N * np.sum(da[:, 1] * db[:, 0] - da[:, 0] * db[:, 1])
beta = (ga.var(axis=0, ddof=1).sum() + lb.var(axis=0, ddof=1).sum()) * N**2 * (N - 1)
D = Ax**2 + Ay**2
u = mb_[0] * Ay + mb_[1] * Ax
v = mb_[0] * Ax - mb_[1] * Ay
expected = np.array([
    [2 / N + beta * (u / D)**2, -beta * u * v / D**2, beta * u / D**1.5],
    [-beta * u * v / D**2, 2 / N + beta * (v / D)**2, -beta * v / D**1.5],
    [beta * u / D**1.5, -beta * v / D**1.5, beta / D]])
check("se2_l2_with_covariance cov", np.allclose(np.asarray(cov), expected, rtol=1e-3, atol=1e-9),
      f"got {np.asarray(cov)}, expected {expected}")
ok, _, _ = se2_l2_with_covariance(TMatchingPairList())
check("se2_l2_with_covariance with no pairs", not ok)

print("se2_l2_robust")
corrs2r = TMatchingPairList()
for c in corrs2:
    corrs2r.push_back(c)
for i in range(6):
    corrs2r.push_back(TMatchingPair(
        20 + i, 20 + i, TPoint3Df(random.uniform(-5, 5), random.uniform(-5, 5), 0),
        TPoint3Df(random.uniform(-5, 5), random.uniform(-5, 5), 0)))
params2 = TSE2RobustParams()
check("TSE2RobustParams defaults", params2.ransac_minSetSize == 3 and
      params2.ransac_maxSetSize == 20 and not params2.verbose and
      close(params2.probability_find_good_model, 0.999, 1e-12))
params2.ransac_maxSetSize = len(corrs2r)
ok, res2 = se2_l2_robust(corrs2r, 0.01, params2)
check("se2_l2_robust ok", ok and isinstance(res2, TSE2RobustResult) and res2.ransac_iters > 0)
check("se2_l2_robust result is a SOG", isinstance(res2.transformation, CPosePDFSOG) and
      len(res2.transformation) >= 1)
_, best = res2.transformation.getMostLikelyCovarianceAndMean()
check("se2_l2_robust pose", close(best.x, 1.0, 1e-3) and close(best.y, 2.0, 1e-3) and
      close(best.phi, 0.3, 1e-4), f"got {best}")
check("se2_l2_robust inliers", sorted(c.globalIdx for c in res2.largestSubSet) == list(range(20)))

print("se3_l2")
q3 = CPose3D(1.0, 2.0, 3.0, 0.3, -0.2, 0.1)
pts_other = [TPoint3D(random.uniform(-5, 5), random.uniform(-5, 5), random.uniform(-5, 5))
             for _ in range(15)]
pts_this = [q3.composePoint(p) for p in pts_other]


def pose3_close(a, b, tol):
    return np.allclose(np.asarray(a.getHomogeneousMatrix()),
                       np.asarray(b.getHomogeneousMatrix()), atol=tol)


ok, est3, scale = se3_l2(pts_this, pts_other)
check("se3_l2 (point lists)", ok and pose3_close(est3, q3, 1e-9) and close(scale, 1.0, 1e-9),
      f"got {est3}, scale={scale}")
ok, _, scale = se3_l2(pts_this, [TPoint3D(2 * p.x, 2 * p.y, 2 * p.z) for p in pts_other])
check("se3_l2 scale", ok and close(scale, 0.5, 1e-9), f"got {scale}")

corrs3 = TMatchingPairList()
corrs3_d = TMatchingPairList_d()
for i, (pt, po) in enumerate(zip(pts_this, pts_other)):
    corrs3.push_back(TMatchingPair(i, i, TPoint3Df(pt.x, pt.y, pt.z), TPoint3Df(po.x, po.y, po.z)))
    corrs3_d.push_back(TMatchingPair_d(i, i, pt, po))
ok, est3, scale = se3_l2(corrs3)
check("se3_l2 (TMatchingPairList)", ok and pose3_close(est3, q3, 1e-4), f"got {est3}")
ok, est3, scale = se3_l2(corrs3_d, forceScaleToUnity=True)
check("se3_l2 (TMatchingPairList_d)", ok and pose3_close(est3, q3, 1e-9) and close(scale, 1, 1e-12),
      f"got {est3}")

print("TSE3RobustParams / se3_l2_robust")
params3 = TSE3RobustParams()
check("TSE3RobustParams defaults", close(params3.ransac_threshold_ang, np.deg2rad(1.0), 1e-12) and
      close(params3.ransac_threshold_scale, 0.03, 1e-12) and params3.forceScaleToUnity and
      not params3.verbose)
params3.ransac_threshold_ang = 0.01
params3.ransac_threshold_scale = 0.05
params3.forceScaleToUnity = False
params3.verbose = False
check("TSE3RobustParams setters", close(params3.ransac_threshold_ang, 0.01, 1e-12) and
      close(params3.ransac_threshold_scale, 0.05, 1e-12) and not params3.forceScaleToUnity)
params3.forceScaleToUnity = True
for i in range(3):
    corrs3.push_back(TMatchingPair(
        15 + i, 15 + i, TPoint3Df(random.uniform(-5, 5), 0, 0), TPoint3Df(0, random.uniform(-5, 5), 0)))
ok, res3 = se3_l2_robust(corrs3, params3)
check("se3_l2_robust", ok and pose3_close(res3.transformation, q3, 1e-3) and
      sorted(res3.inliers_idx) == list(range(15)), f"got {res3.transformation}")

print(f"\nResults: {PASS} passed, {FAIL} failed")
sys.exit(1 if FAIL else 0)
