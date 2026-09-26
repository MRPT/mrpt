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
check("dijkstra_nodes_estimate", abs(gd.getNodePose(2).x - 2.0) < 1e-12
      and abs(gd.getNodePose(4).x - 4.0) < 1e-12, f"got {gd.getNodePose(4)}")

print(f"\nResults: {PASS} passed, {FAIL} failed")
sys.exit(1 if FAIL else 0)
