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

print(f"\nResults: {PASS} passed, {FAIL} failed")
sys.exit(1 if FAIL else 0)
