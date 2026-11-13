"""
mrpt.graphs — Pose graph types and graph algorithms.

Provides:
  - CNetworkOfPoses2D : 2D pose graph (CPose2D edges)
  - CNetworkOfPoses3D : 3D pose graph (CPose3D edges)
  - CNetworkOfPoses2DCov, CNetworkOfPoses3DCov : pose graphs with Gaussian edges
  - CNetworkOfPoses2DInf, CNetworkOfPoses3DInf : pose graphs with edges in
    information form

Example::

    import mrpt.graphs as graphs
    import mrpt.poses as poses

    g = graphs.CNetworkOfPoses2D()
    g.setNodePose(0, poses.CPose2D(0, 0, 0))
    g.setNodePose(1, poses.CPose2D(1, 0, 0))
    g.insertEdge(0, 1, poses.CPose2D(1, 0, 0))
    print(g)
"""
from __future__ import annotations
import mrpt as mrpt
from mrpt.graphs._bindings import CNetworkOfPoses2D as CNetworkOfPoses2D
from mrpt.graphs._bindings import CNetworkOfPoses2DCov as CNetworkOfPoses2DCov
from mrpt.graphs._bindings import CNetworkOfPoses2DInf as CNetworkOfPoses2DInf
from mrpt.graphs._bindings import CNetworkOfPoses3D as CNetworkOfPoses3D
from mrpt.graphs._bindings import CNetworkOfPoses3DCov as CNetworkOfPoses3DCov
from mrpt.graphs._bindings import CNetworkOfPoses3DInf as CNetworkOfPoses3DInf
from . import _bindings
__all__: list = ['CNetworkOfPoses2D', 'CNetworkOfPoses3D', 'CNetworkOfPoses2DCov', 'CNetworkOfPoses3DCov', 'CNetworkOfPoses2DInf', 'CNetworkOfPoses3DInf']
