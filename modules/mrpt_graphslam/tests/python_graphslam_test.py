#!/usr/bin/env python3
"""Unit tests for the mrpt.graphslam Python bindings."""
import math
import random
import sys

try:
    from mrpt.graphslam import TResultInfoSpaLevMarq, optimize_graph_spa_levmarq
except ImportError as e:
    msg = str(e)
    if "_bindings" in msg and "No module named" in msg:
        print(f"SKIP: mrpt.graphslam bindings not built ({e})", file=sys.stderr)
        sys.exit(0)
    print(f"FAIL: mrpt.graphslam import error: {e}", file=sys.stderr)
    sys.exit(1)

import numpy as np
from mrpt.graphs import (
    CNetworkOfPoses2D, CNetworkOfPoses3D, CNetworkOfPoses2DInf, CNetworkOfPoses3DInf,
)
from mrpt.poses import CPose2D, CPose3D, CPosePDFGaussianInf, CPose3DPDFGaussianInf

PASS = FAIL = 0


def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        print(f"  PASS  {name}")
        PASS += 1
    else:
        print(f"  FAIL  {name}" + (f": {detail}" if detail else ""))
        FAIL += 1


def ring_2d(n=12, noise=0.1, seed=0):
    """A ring of n poses with exact edges, and noisy initial node poses."""
    rnd = random.Random(seed)
    gt = [CPose2D(0, 0, 0)]
    for _ in range(1, n):
        gt.append(gt[-1] + CPose2D(1.0, 0, 2 * math.pi / n))
    g = CNetworkOfPoses2D()
    for i in range(n):
        j = (i + 1) % n
        g.insertEdge(i, j, gt[j] - gt[i])
    for i in range(n):
        noisy = CPose2D(rnd.gauss(0, noise), rnd.gauss(0, noise), rnd.gauss(0, noise / 2))
        g.setNodePose(i, gt[i] + noisy if i > 0 else gt[i])
    return g, gt


def max_error_2d(g, gt):
    return max(max(abs(g.getNodePose(i).x - p.x), abs(g.getNodePose(i).y - p.y),
                   abs(g.getNodePose(i).phi - p.phi)) for i, p in enumerate(gt))


print("2D ring with a loop closure")
g, gt = ring_2d()
check("TResultInfoSpaLevMarq", isinstance(TResultInfoSpaLevMarq(), TResultInfoSpaLevMarq))
before = [g.getNodePose(i).x for i in range(12)]
info0 = optimize_graph_spa_levmarq(g, extra_params={"max_iterations": 0})
check("max_iterations=0 leaves the graph unchanged",
      [g.getNodePose(i).x for i in range(12)] == before)
check("initial error", info0.final_total_sq_error > 0.01, f"{info0}")
calls = []
info = optimize_graph_spa_levmarq(
    g, feedback=lambda graph, it, max_it, err: calls.append((graph is g, it, max_it, err)))
check("optimized error", info.final_total_sq_error < 1e-10, f"{info}")
check("optimized poses match the ground truth", max_error_2d(g, gt) < 1e-5,
      f"max error {max_error_2d(g, gt)}")
check("the root node is fixed", g.getNodePose(0).x == 0.0 and g.getNodePose(0).phi == 0.0)
check("feedback is called on each iteration with the graph",
      1 <= len(calls) <= info.num_iters + 1 and all(c[0] for c in calls) and
      [c[1] for c in calls] == list(range(len(calls))) and calls[0][2] == 100 and
      abs(calls[0][3] - info0.final_total_sq_error) < 1e-12, f"{calls}")
check("the error does not increase", all(b[3] <= a[3] for a, b in zip(calls, calls[1:])))



def failing_feedback(graph, it, max_it, err):
    raise RuntimeError("stop requested")


try:
    optimize_graph_spa_levmarq(ring_2d(seed=4)[0], feedback=failing_feedback)
    check("exceptions in the feedback propagate", False, "no exception")
except RuntimeError as e:
    check("exceptions in the feedback propagate", "stop requested" in str(e), str(e))

print("Subset of nodes")
g, gt = ring_2d(seed=1)
fixed = {i: g.getNodePose(i) for i in range(12)}
optimize_graph_spa_levmarq(g, nodes_to_optimize={3, 4, 5})
moved = [i for i in range(12) if g.getNodePose(i).x != fixed[i].x]
check("only the given nodes move", moved and set(moved) <= {3, 4, 5}, f"moved: {moved}")

print("Parameters")
g, gt = ring_2d(seed=2)
info = optimize_graph_spa_levmarq(
    g, extra_params={"max_iterations": 2, "verbose": False, "tau": 1e-3, "e1": 1e-9, "e2": 1e-9,
                     "initial_lambda": 0, "profiler": False})
check("max_iterations", info.num_iters <= 1, f"{info}")
for bad, exc in [({"max_iteration": 10}, KeyError), ({"tau": "big"}, TypeError)]:
    try:
        optimize_graph_spa_levmarq(g, extra_params=bad)
        check(f"bad parameters raise {exc.__name__}", False, "no exception")
    except exc:
        check(f"bad parameters raise {exc.__name__}", True)
try:
    optimize_graph_spa_levmarq(g, nodes_to_optimize=set())
    check("no nodes to optimize raises", False, "no exception")
except ValueError:
    check("no nodes to optimize raises", True)

print("3D graph")
gt3 = [CPose3D(0, 0, 0, 0, 0, 0)]
for k in range(1, 8):
    gt3.append(gt3[-1] + CPose3D(1.0, 0.1, 0.2, 0.3, 0.05, -0.05))
g3 = CNetworkOfPoses3D()
for i in range(7):
    g3.insertEdge(i, i + 1, gt3[i + 1] - gt3[i])
g3.insertEdge(0, 7, gt3[7] - gt3[0])  # loop closure
rnd = random.Random(3)
for i, p in enumerate(gt3):
    g3.setNodePose(i, p + CPose3D(*[rnd.gauss(0, 0.05) for _ in range(6)]) if i else p)
info = optimize_graph_spa_levmarq(g3)
err3 = max(np.abs(np.asarray(g3.getNodePose(i).getHomogeneousMatrix()) -
                  np.asarray(p.getHomogeneousMatrix())).max() for i, p in enumerate(gt3))
check("3D optimized poses", info.final_total_sq_error < 1e-10 and err3 < 1e-5, f"{info}, {err3}")


def two_edges(graph_cls, edge_a, edge_b, pose_cls):
    g = graph_cls()
    g.insertEdge(0, 1, edge_a)
    g.insertEdge(0, 1, edge_b)
    g.setNodePose(0, pose_cls())
    g.setNodePose(1, pose_cls())
    return g


print("Edges with information matrices: weighted mean")
# Two edges 0->1 with x=1 (weight 1) and x=2 (weight 3): the optimum is x=1.75,
# with a total error 1*0.75^2 + 3*0.25^2 = 0.75.
# pybind11 < 2.12 cannot take NumPy >= 2 arrays (mrpt.rtti warns about it):
import mrpt.rtti._bindings as _rtti
_numpy_to_mrpt_ok = not (int(np.__version__.split(".")[0]) >= 2 and
                         tuple(_rtti._pybind11_version[:2]) < (2, 12))
if not _numpy_to_mrpt_ok:
    print("  SKIP  NumPy >= 2 with pybind11 < 2.12")
else:
    g2i = two_edges(CNetworkOfPoses2DInf,
                    CPosePDFGaussianInf(CPose2D(1, 0, 0), np.eye(3)),
                    CPosePDFGaussianInf(CPose2D(2, 0, 0), 3 * np.eye(3)), CPose2D)
    info = optimize_graph_spa_levmarq(g2i)
    check("2DInf weighted mean", abs(g2i.getNodePose(1).x - 1.75) < 1e-6 and
          abs(info.final_total_sq_error - 0.75) < 1e-6, f"{g2i.getNodePose(1)}, {info}")
    g3i = two_edges(CNetworkOfPoses3DInf,
                    CPose3DPDFGaussianInf(CPose3D(1, 0, 0, 0, 0, 0), np.eye(6)),
                    CPose3DPDFGaussianInf(CPose3D(2, 0, 0, 0, 0, 0), 3 * np.eye(6)), CPose3D)
    info = optimize_graph_spa_levmarq(g3i)
    check("3DInf weighted mean", abs(g3i.getNodePose(1).x - 1.75) < 1e-6 and
          abs(info.final_total_sq_error - 0.75) < 1e-6, f"{g3i.getNodePose(1)}, {info}")
g2 = two_edges(CNetworkOfPoses2D, CPose2D(1, 0, 0), CPose2D(2, 0, 0), CPose2D)
info = optimize_graph_spa_levmarq(g2)
check("edges without uncertainty: unweighted mean", abs(g2.getNodePose(1).x - 1.5) < 1e-6 and
      abs(info.final_total_sq_error - 0.5) < 1e-6, f"{g2.getNodePose(1)}, {info}")

print(f"\nResults: {PASS} passed, {FAIL} failed")
sys.exit(1 if FAIL else 0)
