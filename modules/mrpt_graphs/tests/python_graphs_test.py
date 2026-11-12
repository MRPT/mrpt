#!/usr/bin/env python3
"""Smoke tests for mrpt.graphs Python bindings."""
import sys, math

try:
    from mrpt.graphs import CNetworkOfPoses2D, CNetworkOfPoses3D
except ImportError as e:
    msg = str(e)
    if "_bindings" in msg and "No module named" in msg:
        print(f"SKIP: mrpt.graphs bindings not built ({e})", file=sys.stderr)
        sys.exit(0)
    print(f"FAIL: mrpt.graphs import error: {e}", file=sys.stderr)
    sys.exit(1)

PASS = FAIL = 0

def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        print(f"  PASS  {name}"); PASS += 1
    else:
        print(f"  FAIL  {name}" + (f": {detail}" if detail else "")); FAIL += 1

print("CNetworkOfPoses2D")
try:
    from mrpt.poses import CPose2D, CPose3D
    g = CNetworkOfPoses2D()
    g.setNodePose(0, CPose2D(0.0, 0.0, 0.0))
    g.setNodePose(1, CPose2D(1.0, 0.0, 0.0))
    g.setNodePose(2, CPose2D(1.0, 1.0, 0.0))
    check("nodeCount 2D", g.nodeCount() == 3, f"got {g.nodeCount()}")
    g.insertEdge(0, 1, CPose2D(1.0, 0.0, 0.0))
    g.insertEdge(1, 2, CPose2D(0.0, 1.0, 0.0))
    check("edgeCount 2D", g.edgeCount() == 2, f"got {g.edgeCount()}")

    print("CNetworkOfPoses3D")
    g3 = CNetworkOfPoses3D()
    g3.setNodePose(0, CPose3D(0, 0, 0, 0, 0, 0))
    g3.setNodePose(1, CPose3D(1, 0, 0, 0, 0, 0))
    check("nodeCount 3D", g3.nodeCount() == 2, f"got {g3.nodeCount()}")
except ImportError:
    print("  SKIP  (mrpt.poses not available)")

print("Dijkstra")
from mrpt.graphs import CNetworkOfPoses2D as _G2
from mrpt.poses import CPose2D as _CP2
gd = _G2()
for i in range(5):
    gd.setNodePose(i, _CP2(0.0, 0.0, 0.0))
for i in range(4):
    gd.insertEdge(i, i + 1, _CP2(1.0, 0.0, 0.0))
gd.insertEdge(4, 1, _CP2(-3.0, 0.0, 0.0))  # reversed shortcut
check("dijkstra_path uses shortcut", gd.dijkstra_path(0, 4) == [0, 1, 4], f"got {gd.dijkstra_path(0, 4)}")
check("dijkstra_path reverse", gd.dijkstra_path(4, 0) == [4, 1, 0])
dists = gd.getNodeDistances(0)
check("topological distances", dists == {0: 0.0, 1: 1.0, 2: 2.0, 3: 3.0, 4: 2.0}, f"got {dists}")
check("neighbors", gd.getNeighborsOf(1) == {0, 2, 4})
gd.root = 0
gd.dijkstra_nodes_estimate()
gdis = _G2()
gdis.insertEdge(0, 1, _CP2(1.0, 0.0, 0.0))
gdis.insertEdge(5, 6, _CP2(1.0, 0.0, 0.0))  # a separate component
check("disconnected: distances of reachable nodes", gdis.getNodeDistances(0) == {0: 0.0, 1: 1.0})
check("disconnected: path within component", gdis.dijkstra_path(0, 1) == [0, 1])
try:
    gdis.dijkstra_path(0, 6)
    check("disconnected: unreachable target raises", False)
except ValueError:
    check("disconnected: unreachable target raises", True)
try:
    gdis.dijkstra_nodes_estimate()
    check("disconnected: estimate raises", False)
except RuntimeError as ex:
    check("disconnected: estimate raises with message", "not fully connected" in str(ex),
          f"got {str(ex)!r}")
gedges = _G2()  # edges only, no node poses yet
for i in range(3):
    gedges.insertEdge(i, i + 1, _CP2(1.0, 0.0, 0.0))
gedges.dijkstra_nodes_estimate()
check("estimate with no initial node poses", abs(gedges.getNodePose(3).x - 3.0) < 1e-12)
check("dijkstra_nodes_estimate", abs(gd.getNodePose(2).x - 2.0) < 1e-12
      and abs(gd.getNodePose(4).x - 4.0) < 1e-12, f"got {gd.getNodePose(4)}")

print("Reading edges back")
ge = _G2()
ge.insertEdge(2, 3, _CP2(0.5, 0.0, 0.1))
ge.insertEdge(0, 1, _CP2(1.0, 2.0, 0.3))
ge.insertEdge(0, 1, _CP2(9.0, 0.0, 0.0))  # a second, parallel edge
edges = ge.getEdges()
check("getEdges", [(a, b) for a, b, _ in edges] == [(0, 1), (0, 1), (2, 3)] and
      abs(edges[0][2].y - 2.0) < 1e-12 and abs(edges[2][2].phi - 0.1) < 1e-12, f"got {edges}")
check("getEdge", abs(ge.getEdge(2, 3).x - 0.5) < 1e-12 and abs(ge.getEdge(0, 1).x - 1.0) < 1e-12)
check("edgeExists", ge.edgeExists(0, 1) and not ge.edgeExists(1, 0))
try:
    ge.getEdge(1, 0)
    check("getEdge of a missing edge raises", False)
except KeyError:
    check("getEdge of a missing edge raises", True)
ge.eraseEdge(0, 1)
check("eraseEdge", ge.edgeCount() == 1 and not ge.edgeExists(0, 1))
ge.clearEdges()
check("clearEdges", ge.edgeCount() == 0 and ge.getEdges() == [])

print("Pose graphs with Gaussian edges")
import numpy as np
from mrpt.graphs import (
    CNetworkOfPoses2DCov, CNetworkOfPoses3DCov, CNetworkOfPoses2DInf, CNetworkOfPoses3DInf,
)
from mrpt.poses import (
    CPosePDFGaussian, CPose3DPDFGaussian, CPosePDFGaussianInf, CPose3DPDFGaussianInf,
)
# pybind11 < 2.12 cannot take NumPy >= 2 arrays (mrpt.rtti warns about it):
import mrpt.rtti._bindings as _rtti
_numpy_to_mrpt_ok = not (int(np.__version__.split(".")[0]) >= 2 and
                         tuple(_rtti._pybind11_version[:2]) < (2, 12))
if not _numpy_to_mrpt_ok:
    print("  SKIP  NumPy >= 2 with pybind11 < 2.12")
else:
    g2c = CNetworkOfPoses2DCov()
    g2c.insertEdge(0, 1, CPosePDFGaussian(_CP2(1.0, 0.0, math.pi / 2), np.eye(3) * 0.01))
    g2c.insertEdge(1, 2, CPosePDFGaussian(_CP2(1.0, 0.0, 0.0), np.eye(3) * 0.02))
    g2c.dijkstra_nodes_estimate()
    p2 = g2c.getNodePose(2)
    check("2DCov node estimate", isinstance(p2, _CP2) and abs(p2.x - 1.0) < 1e-9 and
          abs(p2.y - 1.0) < 1e-9, f"got {p2}")
    e = g2c.getEdge(1, 2)
    check("2DCov edge read back", isinstance(e, CPosePDFGaussian) and
          abs(np.asarray(e.cov)[0, 0] - 0.02) < 1e-12)
    g2i = CNetworkOfPoses2DInf()
    g2i.insertEdge(0, 1, CPosePDFGaussianInf(_CP2(2.0, 0.0, 0.0), np.eye(3) * 100.0))
    check("2DInf edge read back", abs(np.asarray(g2i.getEdges()[0][2].cov_inv)[1, 1] - 100.0) < 1e-9)
    g2i.setNodePose(5, _CP2(1.0, 2.0, 3.0))
    check("2DInf node pose", abs(g2i.getNodePose(5).y - 2.0) < 1e-12)
    g3c = CNetworkOfPoses3DCov()
    g3c.insertEdge(0, 1, CPose3DPDFGaussian(CPose3D(1, 0, 0, 0, 0, 0), np.eye(6) * 0.01))
    g3c.dijkstra_nodes_estimate()
    check("3DCov node estimate", abs(g3c.getNodePose(1).x - 1.0) < 1e-9)
    g3i = CNetworkOfPoses3DInf()
    g3i.insertEdge(0, 1, CPose3DPDFGaussianInf(CPose3D(0, 2, 0, 0, 0, 0), np.eye(6)))
    g3i.dijkstra_nodes_estimate()
    check("3DInf node estimate", abs(g3i.getNodePose(1).y - 2.0) < 1e-9 and
          isinstance(g3i.getEdge(0, 1), CPose3DPDFGaussianInf))

print(f"\nResults: {PASS} passed, {FAIL} failed")
sys.exit(1 if FAIL else 0)
