#!/usr/bin/env python3
"""
Unit test for the mrpt.poses Python bindings.

Run via CTest (mrpt_add_python_binding_test) or directly:
  python3 modules/mrpt_poses/tests/python_poses_test.py
"""
import math
import sys

# ---------------------------------------------------------------------------
# Import
# ---------------------------------------------------------------------------
try:
    from mrpt.poses import (
        CPose2D, CPose3D,
        CPose3DPDFGaussian,
        SE_average2, SE_average3,
        CPoint2D, CPoint3D,
    )
except ImportError as e:
    msg = str(e)
    # Soft-skip only when the compiled extension simply isn't present
    # (e.g. bindings disabled at build time).  Any other ImportError,
    # including circular imports caused by a bad PYTHONPATH setup, is
    # a real failure and must surface as a test failure.
    if "_bindings" in msg and "No module named" in msg:
        print(f"SKIP: mrpt.poses bindings not built ({e})", file=sys.stderr)
        sys.exit(0)
    print(f"FAIL: mrpt.poses import error: {e}", file=sys.stderr)
    sys.exit(1)

import numpy as np

PASS = 0
FAIL = 0

def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        print(f"  PASS  {name}")
        PASS += 1
    else:
        print(f"  FAIL  {name}" + (f": {detail}" if detail else ""))
        FAIL += 1

# ---------------------------------------------------------------------------
# CPose2D
# ---------------------------------------------------------------------------
print("CPose2D")
p = CPose2D(1.0, 2.0, math.pi / 4)
check("x attr",      abs(p.x - 1.0) < 1e-9)
check("y attr",      abs(p.y - 2.0) < 1e-9)
check("phi attr",    abs(p.phi - math.pi / 4) < 1e-9)

p2 = CPose2D(0.0, 0.0, 0.0)
inv = p + p.inverse()
check("p + inv ≈ 0 (x)", abs(inv.x) < 1e-6)
check("p + inv ≈ 0 (y)", abs(inv.y) < 1e-6)

p.x = 5.0
check("x setter", abs(p.x - 5.0) < 1e-9)

check("norm",  p.norm() > 0)

# composition
a = CPose2D(1.0, 0.0, 0.0)
b = CPose2D(1.0, 0.0, 0.0)
c = a + b
check("compose x", abs(c.x - 2.0) < 1e-9)

# ---------------------------------------------------------------------------
# CPose3D
# ---------------------------------------------------------------------------
print("CPose3D")
p3 = CPose3D.FromXYZYawPitchRoll(1.0, 2.0, 3.0, 0.0, 0.0, 0.0)
check("x",  abs(p3.x - 1.0) < 1e-9)
check("y",  abs(p3.y - 2.0) < 1e-9)
check("z",  abs(p3.z - 3.0) < 1e-9)

p3.z = 99.0
check("z setter", abs(p3.z - 99.0) < 1e-9)

rot = np.array(p3.getRotationMatrix())
check("rotation matrix shape", rot.shape == (3, 3),
      f"got {rot.shape}")

# identity rotation → diagonal 1
p_id = CPose3D(0, 0, 0, 0, 0, 0)
r_id = np.array(p_id.getRotationMatrix())
check("identity rotation matrix",
      abs(float(r_id[0, 0]) - 1.0) < 1e-6 and
      abs(float(r_id[1, 1]) - 1.0) < 1e-6 and
      abs(float(r_id[2, 2]) - 1.0) < 1e-6)

pt = p_id.inverseComposePoint(1.0, 2.0, 3.0)
check("inverseComposePoint returns TPoint3D", hasattr(pt, 'x') and hasattr(pt, 'y') and hasattr(pt, 'z'), str(type(pt)))
check("inverseComposePoint at identity", abs(pt.x - 1.0) < 1e-6)

# ---------------------------------------------------------------------------
# CPoint2D / CPoint3D
# ---------------------------------------------------------------------------
print("CPoint2D / CPoint3D")
pt2 = CPoint2D(3.0, 4.0)
check("CPoint2D x", abs(pt2.x - 3.0) < 1e-9)
check("CPoint2D y", abs(pt2.y - 4.0) < 1e-9)

pt3 = CPoint3D(1.0, 2.0, 3.0)
check("CPoint3D z", abs(pt3.z - 3.0) < 1e-9)

# ---------------------------------------------------------------------------
# CPose3DPDFGaussian
# ---------------------------------------------------------------------------
print("CPose3DPDFGaussian")
mean = CPose3D(1.0, 2.0, 3.0, 0, 0, 0)
pdf = CPose3DPDFGaussian(mean)
check("PDF mean x", abs(pdf.mean.x - 1.0) < 1e-9)

pdf.mean = CPose3D(10.0, 0.0, 0.0, 0, 0, 0)
check("PDF mean setter", abs(pdf.mean.x - 10.0) < 1e-9)

sample = pdf.drawSingleSample()
check("drawSingleSample returns CPose3D",
      hasattr(sample, "x") and hasattr(sample, "y"))

# ---------------------------------------------------------------------------
# SE_average2 / SE_average3
# ---------------------------------------------------------------------------
print("SE_average2")
avg2 = SE_average2()
avg2.append(CPose2D(2.0, 0.0, 0.0))
avg2.append(CPose2D(4.0, 0.0, 0.0))
r2 = avg2.get_average()
check("SE_average2 mean x ≈ 3", abs(r2.x - 3.0) < 0.1,
      f"got x={r2.x}")

print("SE_average3")
avg3 = SE_average3()
avg3.append(CPose3D(10.0,  0.0, 0.0, 0, 0, 0))
avg3.append(CPose3D(10.2,  0.0, 0.0, 0, 0, 0))
avg3.append(CPose3D( 9.8,  0.0, 0.0, 0, 0, 0))
r3 = avg3.get_average()
check("SE_average3 mean x ≈ 10", abs(r3.x - 10.0) < 0.2,
      f"got x={r3.x}")

# ---------------------------------------------------------------------------
# Particle PDFs and PDF base methods
# ---------------------------------------------------------------------------
print("CPosePDFParticles")
from mrpt.poses import CPosePDFParticles, CPose3DPDFParticles, CPose2D as _P2
from mrpt.math import TPose2D, TPose3D
from mrpt.bayes import CParticleFilterCapable
parts = CPosePDFParticles(100)
check("particles count", len(parts) == 100 and parts.particlesCount() == 100)
check("is CParticleFilterCapable", isinstance(parts, CParticleFilterCapable))
parts.resetDeterministic(TPose2D(1.0, 2.0, 0.5))
mean = parts.getMean()
check("deterministic mean", abs(mean.x - 1.0) < 1e-9 and abs(mean.y - 2.0) < 1e-9,
      f"got {mean}")
cov, mean2 = parts.getCovarianceAndMean()
import numpy as np
check("deterministic cov ~0", np.allclose(np.asarray(cov), 0.0, atol=1e-12))
check("getCovarianceAndMean mean", isinstance(mean2, _P2) and abs(mean2.phi - 0.5) < 1e-9)
parts.resetUniform(-1.0, 1.0, -2.0, 2.0, particlesCount=500)
arr = parts.getParticlesAsNumpy()
check("particles array shape", arr.shape == (500, 4), f"got {arr.shape}")
check("uniform x range", arr[:, 0].min() >= -1.0 and arr[:, 0].max() <= 1.0)
check("uniform y spread", arr[:, 1].max() - arr[:, 1].min() > 3.0)
parts.setW(0, 5.0)
check("setW/getW", abs(parts.getW(0) - 5.0) < 1e-12)
p0 = parts.getParticlePose(0)
best = parts.getMostLikelyParticle()
check("most likely particle", abs(best.x - p0.x) < 1e-12 and abs(best.y - p0.y) < 1e-12)

print("CPose3DPDFParticles")
p3 = CPose3DPDFParticles(10)
p3.resetDeterministic(TPose3D(1.0, 2.0, 3.0, 0.1, 0.2, 0.3))
m3 = p3.getMean()
check("3D mean", abs(m3.z - 3.0) < 1e-9, f"got {m3}")
check("3D particles array", p3.getParticlesAsNumpy().shape == (10, 7))

print("CPose3DPDFGaussian base methods")
g3 = CPose3DPDFGaussian(CPose3D(1.0, 0.0, 0.0, 0.0, 0.0, 0.0))
check("Gaussian getMean via base", abs(g3.getMean().x - 1.0) < 1e-12)

print("NumPy arrays as MRPT matrices (implicit conversion)")
# pybind11 < 2.12 cannot take NumPy >= 2 arrays (mrpt.rtti warns about it):
import mrpt.rtti._bindings as _rtti
if int(np.__version__.split(".")[0]) >= 2 and tuple(_rtti._pybind11_version[:2]) < (2, 12):
    print("  SKIP  NumPy >= 2 with pybind11 < 2.12")
else:
    g6 = CPose3DPDFGaussian(CPose3D(1.0, 0.0, 0.0, 0.0, 0.0, 0.0), np.eye(6) * 0.01)
    check("6x6 covariance from numpy", abs(np.asarray(g6.cov)[2, 2] - 0.01) < 1e-12)
    try:
        CPose3DPDFGaussian(CPose3D(), np.eye(5))
        check("wrong shape rejected", False, "no exception")
    except TypeError:
        check("wrong shape rejected", True)

# ---------------------------------------------------------------------------
# Inverses: CPose2D.inverse() returns a new pose, CPose3D.inverse() works in
# place, and unary minus always returns a new pose.
# ---------------------------------------------------------------------------
import copy
import datetime
from mrpt.math import TPoint2D, TPoint3D, TPose3DQuat
from mrpt.poses import (
    CPose3DQuat, CPosePDFGaussian, CPosePDFGaussianInf, CPose3DPDFGaussianInf,
    CPosePDFSOG, CPose2DInterpolator, CPose3DInterpolator, TInterpolatorMethod,
    imLinearSlerp, imSpline,
)


def close(a, b, tol=1e-9):
    return abs(a - b) < tol


def pose2d_close(a, b, tol=1e-9):
    return close(a.x, b.x, tol) and close(a.y, b.y, tol) and close(a.phi, b.phi, tol)


def pose3d_close(a, b, tol=1e-9):
    return np.allclose(np.asarray(a.getHomogeneousMatrix()),
                       np.asarray(b.getHomogeneousMatrix()), atol=tol)


print("Inverses")
p = CPose2D(1.0, 2.0, 0.5)
n = -p
check("CPose2D -p == p.inverse()", pose2d_close(n, p.inverse()))
check("CPose2D -p value", pose2d_close(
    n, CPose2D(-math.cos(0.5) - 2 * math.sin(0.5), math.sin(0.5) - 2 * math.cos(0.5), -0.5)),
    f"got {n}")
check("CPose2D -p does not modify p", pose2d_close(p, CPose2D(1.0, 2.0, 0.5)))
check("CPose2D p + (-p) == 0", pose2d_close(p + n, CPose2D(0, 0, 0)))

q = CPose3D(1.0, 2.0, 3.0, 0.1, 0.2, 0.3)
nq = -q
check("CPose3D -q does not modify q", pose3d_close(q, CPose3D(1.0, 2.0, 3.0, 0.1, 0.2, 0.3)))
check("CPose3D q + (-q) == identity", pose3d_close(q + nq, CPose3D()))
check("CPose3D -q matrix", np.allclose(np.asarray(nq.getHomogeneousMatrix()),
                                        np.asarray(q.getInverseHomogeneousMatrix())))
q_inplace = CPose3D(q)
ret = q_inplace.inverse()
check("CPose3D.inverse() works in place", ret is None and pose3d_close(q_inplace, nq))

qq = CPose3DQuat(q)
nqq = CPose3DQuat(nq)
check("CPose3DQuat -q", np.allclose([(-qq).x, (-qq).y, (-qq).z], [nqq.x, nqq.y, nqq.z]))
check("CPose3DQuat -q does not modify q", close(qq.x, 1.0) and close(qq.z, 3.0))

print("Constructors from lightweight types")
check("CPose2D(TPose2D)", pose2d_close(CPose2D(TPose2D(1.0, 2.0, 0.3)), CPose2D(1.0, 2.0, 0.3)))
check("CPose3D(TPose3D)", pose3d_close(CPose3D(TPose3D(1.0, 2.0, 3.0, 0.1, 0.2, 0.3)), q))
tq = TPose3DQuat(1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0)
check("CPose3DQuat(TPose3DQuat)", close(CPose3DQuat(tq).z, 3.0))

print("CPose2D composePoint / inverseComposePoint")
c, s_ = math.cos(0.5), math.sin(0.5)
g = p.composePoint(TPoint2D(3.0, -1.0))
check("composePoint(TPoint2D)", close(g.x, 1 + 3 * c + s_) and close(g.y, 2 + 3 * s_ - c),
      f"got {g}")
g2 = p.composePoint(3.0, -1.0)
check("composePoint(x, y)", isinstance(g2, TPoint2D) and close(g2.x, g.x) and close(g2.y, g.y))
g3 = p.composePoint(TPoint3D(3.0, -1.0, 7.0))
check("composePoint(TPoint3D)", isinstance(g3, TPoint3D) and close(g3.x, g.x) and close(g3.z, 7.0))
l = p.inverseComposePoint(g)
check("inverseComposePoint(TPoint2D)", close(l.x, 3.0) and close(l.y, -1.0), f"got {l}")
l2 = p.inverseComposePoint(g.x, g.y)
check("inverseComposePoint(x, y)", close(l2.x, 3.0) and close(l2.y, -1.0), f"got {l2}")
check("CPose2D/CPose3D composePoint agree", close(
    CPose2D(1.0, 2.0, 0.5).composePoint(1.0, 0.0).x,
    CPose3D(1.0, 2.0, 0.0, 0.5, 0.0, 0.0).composePoint(1.0, 0.0, 0.0).x))

print("Copies")
for name, obj, attr in [
    ("CPose2D", CPose2D(1.0, 2.0, 0.5), "x"),
    ("CPose3D", CPose3D(1.0, 2.0, 3.0, 0.1, 0.2, 0.3), "x"),
    ("CPose3DQuat", CPose3DQuat(q), "x"),
    ("CPoint2D", CPoint2D(1.0, 2.0), "x"),
    ("CPoint3D", CPoint3D(1.0, 2.0, 3.0), "x"),
]:
    for how, cp in [("ctor", type(obj)(obj)), ("copy", copy.copy(obj)),
                    ("deepcopy", copy.deepcopy(obj))]:
        setattr(cp, attr, 100.0)
        check(f"{name} {how} is independent",
              type(cp) is type(obj) and close(getattr(obj, attr), 1.0) and
              close(getattr(cp, attr), 100.0))

pdf3 = CPose3DPDFGaussian(q)
pdf3c = copy.deepcopy(pdf3)
pdf3c.mean = CPose3D()
check("CPose3DPDFGaussian deepcopy is independent", close(pdf3.mean.x, 1.0))
for obj in [CPosePDFGaussian(p), CPosePDFGaussianInf(p), CPose3DPDFGaussianInf(q),
            CPosePDFParticles(3), CPose3DPDFParticles(3), CPosePDFSOG(2),
            CPose2DInterpolator(), CPose3DInterpolator()]:
    check(f"{type(obj).__name__} copies",
          type(copy.copy(obj)) is type(obj) and type(copy.deepcopy(obj)) is type(obj) and
          type(type(obj)(obj)) is type(obj))
parts = CPosePDFParticles(5)
parts_c = copy.copy(parts)
parts_c.resetDeterministic(TPose2D(1.0, 0.0, 0.0), 10)
check("CPosePDFParticles copy is independent", len(parts) == 5 and len(parts_c) == 10)

# ---------------------------------------------------------------------------
# CPosePDFGaussian / CPose3DPDFGaussian composition
# ---------------------------------------------------------------------------
print("CPosePDFGaussian composition")


def jacobians_se2(a, b):
    """Jacobians of a (+) b with respect to a and b."""
    ca, sa = math.cos(a.phi), math.sin(a.phi)
    ja = np.array([[1, 0, -b.x * sa - b.y * ca], [0, 1, b.x * ca - b.y * sa], [0, 0, 1]])
    jb = np.array([[ca, -sa, 0], [sa, ca, 0], [0, 0, 1]])
    return ja, jb


if int(np.__version__.split(".")[0]) >= 2 and tuple(_rtti._pybind11_version[:2]) < (2, 12):
    print("  SKIP  NumPy >= 2 with pybind11 < 2.12")
else:
    ma, mb = CPose2D(1.0, 2.0, 0.5), CPose2D(0.5, -0.3, 0.2)
    ca_, cb_ = np.diag([0.01, 0.02, 0.003]), np.diag([0.04, 0.01, 0.002])
    a = CPosePDFGaussian(ma, ca_)
    b = CPosePDFGaussian(mb, cb_)
    ja, jb = jacobians_se2(ma, mb)
    expected_cov = ja @ ca_ @ ja.T + jb @ cb_ @ jb.T

    s = a + b
    check("a + b mean", pose2d_close(s.mean, ma + mb))
    check("a + b cov", np.allclose(np.asarray(s.cov), expected_cov), f"got {np.asarray(s.cov)}")

    a2 = copy.copy(a)
    same = a2
    a2 += b
    check("a += b keeps the object", a2 is same)
    check("a += b", pose2d_close(a2.mean, s.mean) and np.allclose(np.asarray(a2.cov), expected_cov))
    check("a not modified by a2 += b", pose2d_close(a.mean, ma))

    a3 = CPosePDFGaussian(a)
    a3 += mb
    check("a += CPose2D", pose2d_close(a3.mean, ma + mb) and
          np.allclose(np.asarray(a3.cov), ja @ ca_ @ ja.T))

    r = ma + b
    rot = jacobians_se2(ma, mb)[1]
    check("CPose2D + pdf", isinstance(r, CPosePDFGaussian) and pose2d_close(r.mean, ma + mb) and
          np.allclose(np.asarray(r.cov), rot @ cb_ @ rot.T))
    r2 = CPosePDFGaussian(b)
    r2.changeCoordinatesReference(ma)
    check("changeCoordinatesReference(CPose2D)", pose2d_close(r2.mean, r.mean) and
          np.allclose(np.asarray(r2.cov), np.asarray(r.cov)))
    r3 = CPosePDFGaussian(b)
    r3.changeCoordinatesReference(CPose3D(ma))
    check("changeCoordinatesReference(CPose3D)", pose2d_close(r3.mean, r.mean))

    # a - b is the pose of a relative to b, so (a + b) - a == b:
    d = s - a
    check("(a + b) - a mean == b", pose2d_close(d.mean, mb, 1e-9), f"got {d.mean}")
    ms = s.mean
    cp, sp = math.cos(ma.phi), math.sin(ma.phi)
    dx, dy = ms.x - ma.x, ms.y - ma.y
    j_s = np.array([[cp, sp, 0], [-sp, cp, 0], [0, 0, 1]])
    j_a = np.array([[-cp, -sp, -dx * sp + dy * cp], [sp, -cp, -dx * cp - dy * sp], [0, 0, -1]])
    check("(a + b) - a cov", np.allclose(
        np.asarray(d.cov), j_s @ np.asarray(s.cov) @ j_s.T + j_a @ ca_ @ j_a.T))
    d2 = CPosePDFGaussian()
    d2.inverseComposition(s, a)
    check("inverseComposition", pose2d_close(d2.mean, d.mean) and
          np.allclose(np.asarray(d2.cov), np.asarray(d.cov)))
    d3 = CPosePDFGaussian()
    d3.inverseComposition(s, a, np.zeros((3, 3)))
    check("inverseComposition with zero cross-covariance",
          np.allclose(np.asarray(d3.cov), np.asarray(d.cov)))
    d4 = CPosePDFGaussian(s)
    d4 -= a
    check("-=", pose2d_close(d4.mean, d.mean) and np.allclose(np.asarray(d4.cov), np.asarray(d.cov)))

    inv = a.inverse()
    cp, sp = math.cos(ma.phi), math.sin(ma.phi)
    h = np.array([[-cp, -sp, ma.x * sp - ma.y * cp], [sp, -cp, ma.x * cp + ma.y * sp], [0, 0, -1]])
    check("inverse() mean", pose2d_close(inv.mean, -ma))
    check("inverse() cov", np.allclose(np.asarray(inv.cov), h @ ca_ @ h.T))
    check("inverse() does not modify the PDF", pose2d_close(a.mean, ma))

    print("CPose3DPDFGaussian composition")
    qa, qb = CPose3D(1.0, 2.0, 3.0, 0.1, 0.2, 0.3), CPose3D(0.5, -0.3, 0.1, -0.2, 0.1, 0.05)
    A = CPose3DPDFGaussian(qa, np.eye(6) * 1e-4)
    B = CPose3DPDFGaussian(qb, np.eye(6) * 2e-4)
    S = A + B
    check("A + B mean", pose3d_close(S.mean, qa + qb))
    A2 = copy.copy(A)
    same = A2
    A2 += B
    check("A += B keeps the object", A2 is same and pose3d_close(A2.mean, qa + qb) and
          np.allclose(np.asarray(A2.cov), np.asarray(S.cov)))
    A3 = CPose3DPDFGaussian(A)
    same = A3
    A3 += qb
    check("A += CPose3D keeps the object", A3 is same and pose3d_close(A3.mean, qa + qb))
    D = S - A
    check("(A + B) - A mean == B", pose3d_close(D.mean, qb, 1e-6))
    S -= A
    check("-=", pose3d_close(S.mean, qb, 1e-6) and np.allclose(np.asarray(S.cov), np.asarray(D.cov)))
    I = -A
    check("-A mean", pose3d_close(I.mean, -qa))
    check("-A == A.inverse()", pose3d_close(A.inverse().mean, I.mean) and
          np.allclose(np.asarray(A.inverse().cov), np.asarray(I.cov)))
    check("-A does not modify A", pose3d_close(A.mean, qa))
    check("(-A) + A has mean identity", pose3d_close((I + A).mean, CPose3D(), 1e-9))
    C = CPose3DPDFGaussian(B)
    C.changeCoordinatesReference(qa)
    check("changeCoordinatesReference", pose3d_close(C.mean, qa + qb))

# ---------------------------------------------------------------------------
# CPosePDFSOG
# ---------------------------------------------------------------------------
print("CPosePDFSOG")
sog = CPosePDFSOG(2)
check("SOG size", len(sog) == 2 and sog.size() == 2)
sog[0].mean = CPose2D(1.0, 0.0, 0.0)
sog[0].log_w = math.log(0.25)
sog[1].mean = CPose2D(3.0, 0.0, 0.0)
sog[1].log_w = math.log(0.75)
check("SOG mean", close(sog.getMean().x, 0.25 * 1.0 + 0.75 * 3.0), f"got {sog.getMean()}")
cov_ml, mean_ml = sog.getMostLikelyCovarianceAndMean()
check("SOG most likely mode", close(mean_ml.x, 3.0))
check("SOG iteration", [round(mo.mean.x) for mo in sog] == [1, 3])
mode = CPosePDFSOG.TGaussianMode()
mode.mean = CPose2D(5.0, 0.0, 0.0)
sog.push_back(mode)
check("SOG push_back", len(sog) == 3 and close(sog.get(2).mean.x, 5.0))
try:
    sog[3]
    check("SOG index out of range", False, "no exception")
except IndexError:
    check("SOG index out of range", True)

# ---------------------------------------------------------------------------
# Interpolation methods
# ---------------------------------------------------------------------------
print("Interpolation methods")
interp = CPose3DInterpolator()
check("default method", interp.getInterpolationMethod() == imLinearSlerp)
for i in range(4):
    interp.insert(datetime.timedelta(seconds=100 + i), TPose3D(i * i, 0, 0, 0, 0, 0))
t_query = datetime.timedelta(seconds=101.5)
pq, valid = interp.interpolate(t_query)
check("linear interpolation", valid and close(pq.x, 2.5), f"got {pq}")
interp.setInterpolationMethod(TInterpolatorMethod.imSpline)
check("setInterpolationMethod", interp.getInterpolationMethod() == imSpline)
pq, valid = interp.interpolate(t_query)
# Natural cubic spline through (0,0), (1,1), (2,4), (3,9), evaluated at 1.5:
check("spline interpolation", valid and close(pq.x, 2.2), f"got {pq}")

interp2 = CPose2DInterpolator()
for i in range(4):
    interp2.insert(datetime.timedelta(seconds=100 + i), TPose2D(i * i, 0, 0))
interp2.setInterpolationMethod(imSpline)
pq2, valid = interp2.interpolate(t_query)
check("CPose2DInterpolator spline", interp2.getInterpolationMethod() == imSpline and valid and
      close(pq2.x, 2.2), f"got {pq2}")

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
print(f"\nResults: {PASS} passed, {FAIL} failed")
if FAIL:
    sys.exit(1)
