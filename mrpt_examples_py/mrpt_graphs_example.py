#!/usr/bin/env python3
"""
mrpt_graphs_example.py — pose graphs with mrpt.graphs.

Demonstrates:
  - CNetworkOfPoses2D: build, insert nodes/edges, query
  - CNetworkOfPoses3D: same in 3D
  - save/load to text file
"""

import tempfile, os
from mrpt.graphs import CNetworkOfPoses2D, CNetworkOfPoses3D
from mrpt.poses import CPose2D, CPose3D

# ---------------------------------------------------------------------------
# 2D pose graph
# ---------------------------------------------------------------------------
g2 = CNetworkOfPoses2D()

# Add node poses (estimated positions)
g2.setNodePose(0, CPose2D(0.0, 0.0, 0.0))
g2.setNodePose(1, CPose2D(1.0, 0.0, 0.0))
g2.setNodePose(2, CPose2D(1.0, 1.0, 1.5708))  # ~90 deg

# Add odometry edges (relative transforms)
g2.insertEdge(0, 1, CPose2D(1.0,  0.0, 0.0))
g2.insertEdge(1, 2, CPose2D(0.0,  1.0, 1.5708))
g2.insertEdge(2, 0, CPose2D(-1.0, -1.0, -1.5708))  # loop closure

print(f"2D graph: {g2.nodeCount()} nodes, {g2.edgeCount()} edges")
print(f"  node 1 pose: {g2.getNodePose(1)}")
print(f"  node IDs: {g2.getNodeIDs()}")
assert g2.hasNode(0) and g2.hasNode(2)
assert not g2.hasNode(99)

# ---------------------------------------------------------------------------
# Save / load round-trip
# ---------------------------------------------------------------------------
with tempfile.NamedTemporaryFile(suffix=".graph", delete=False) as f:
    fname = f.name
try:
    g2.saveToTextFile(fname)
    g2b = CNetworkOfPoses2D()
    g2b.loadFromTextFile(fname)
    print(f"  After save/load: {g2b.nodeCount()} nodes, {g2b.edgeCount()} edges  ✓")
    assert g2b.nodeCount() == 3
finally:
    os.unlink(fname)

# ---------------------------------------------------------------------------
# 3D pose graph
# ---------------------------------------------------------------------------
g3 = CNetworkOfPoses3D()
g3.setNodePose(0, CPose3D(0, 0, 0, 0, 0, 0))
g3.setNodePose(1, CPose3D(1, 0, 0, 0, 0, 0))
g3.insertEdge(0, 1, CPose3D(1, 0, 0, 0, 0, 0))

print(f"\n3D graph: {g3.nodeCount()} nodes, {g3.edgeCount()} edges")
print(f"  {g3}")
