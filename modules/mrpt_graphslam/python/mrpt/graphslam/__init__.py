"""
mrpt.graphslam: pose-graph optimization.

Provides:
  - optimize_graph_spa_levmarq : Levenberg-Marquardt optimization of the node
    poses of a CNetworkOfPoses2D, CNetworkOfPoses3D, CNetworkOfPoses2DInf or
    CNetworkOfPoses3DInf graph (mrpt.graphs)
  - TResultInfoSpaLevMarq      : Its output information

Example::

    import mrpt.graphs as graphs
    import mrpt.graphslam as graphslam

    g = graphs.CNetworkOfPoses2D()
    g.loadFromTextFile("graph.g2o")
    g.dijkstra_nodes_estimate()
    info = graphslam.optimize_graph_spa_levmarq(g, extra_params={"max_iterations": 50})
    print(info.final_total_sq_error)
"""

import mrpt.rtti           # noqa: F401
import mrpt.serialization  # noqa: F401
import mrpt.poses          # noqa: F401  (node and edge types)
import mrpt.graphs         # noqa: F401  (the pose graphs)

from mrpt.graphslam._bindings import (
    TResultInfoSpaLevMarq,
    optimize_graph_spa_levmarq,
)

__all__ = [
    "TResultInfoSpaLevMarq",
    "optimize_graph_spa_levmarq",
]
