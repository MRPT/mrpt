"""
mrpt.graphs — Pose graph types and graph algorithms.

Provides:
  - CNetworkOfPoses2D : 2D pose graph (CPose2D edges)
  - CNetworkOfPoses3D : 3D pose graph (CPose3D edges)

Example::

    import mrpt.graphs as graphs
    import mrpt.poses as poses

    g = graphs.CNetworkOfPoses2D()
    g.setNodePose(0, poses.CPose2D(0, 0, 0))
    g.setNodePose(1, poses.CPose2D(1, 0, 0))
    g.insertEdge(0, 1, poses.CPose2D(1, 0, 0))
    print(g)
"""

from mrpt.graphs._bindings import (
    CNetworkOfPoses2D,
    CNetworkOfPoses3D,
)

__all__ = [
    "CNetworkOfPoses2D",
    "CNetworkOfPoses3D",
]
