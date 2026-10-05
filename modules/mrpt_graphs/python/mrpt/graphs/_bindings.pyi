"""
Python bindings for mrpt::graphs — pose graph and graph algorithms
"""
from __future__ import annotations
import mrpt.poses
__all__: list[str] = ['CNetworkOfPoses2D', 'CNetworkOfPoses3D']
class CNetworkOfPoses2D:
    """
    A graph of 2D poses (nodes) and relative pose constraints (edges).
    """
    def __init__(self) -> None:
        """
        Creates an empty graph.
        """
    def __repr__(self) -> str:
        ...
    def dijkstra_nodes_estimate(self) -> None:
        """
        Recomputes all node poses by composing edges along the shortest-path spanning tree from the root node
        """
    def dijkstra_path(self, source: int, target: int) -> list[int]:
        """
        Shortest path (fewest edges) between two nodes as a list of node IDs, from source to target, ignoring edge directions. Raises ValueError if target is unreachable.
        """
    def edgeCount(self) -> int:
        """
        Number of edges in the graph
        """
    def getNeighborsOf(self, node_id: int) -> set[int]:
        """
        IDs of the nodes connected to the given one by an edge (in any direction)
        """
    def getNodeDistances(self, source: int) -> dict[int, float]:
        """
        Topological distance (number of edges) from source to every reachable node
        """
    def getNodeIDs(self) -> list[int]:
        """
        Return a list of all node IDs
        """
    def getNodePose(self, node_id: int) -> mrpt.poses.CPose2D:
        """
        Get the estimated pose for a node
        """
    def hasNode(self, node_id: int) -> bool:
        """
        True if a node with the given ID exists
        """
    def insertEdge(self, from_id: int, to_id: int, edge: mrpt.poses.CPose2D) -> None:
        """
        Insert a directed edge from → to
        """
    def loadFromTextFile(self, fileName: str, collapse_dup_edges: bool = True) -> None:
        """
        Loads the graph from a text file in the TORO / g2o format.
        """
    def nodeCount(self) -> int:
        """
        Number of nodes in the graph
        """
    def saveToTextFile(self, fileName: str) -> None:
        """
        Saves the graph to a text file in the TORO / g2o format.
        """
    def setNodePose(self, node_id: int, pose: mrpt.poses.CPose2D) -> None:
        """
        Set the estimated pose for a node
        """
    @property
    def root(self) -> int:
        """
        Root node ID (default: 0)
        """
    @root.setter
    def root(self, arg0: int) -> None:
        ...
class CNetworkOfPoses3D:
    """
    A graph of 3D poses (nodes) and relative pose constraints (edges).
    """
    def __init__(self) -> None:
        """
        Creates an empty graph.
        """
    def __repr__(self) -> str:
        ...
    def dijkstra_nodes_estimate(self) -> None:
        """
        Recomputes all node poses by composing edges along the shortest-path spanning tree from the root node
        """
    def dijkstra_path(self, source: int, target: int) -> list[int]:
        """
        Shortest path (fewest edges) between two nodes as a list of node IDs, from source to target, ignoring edge directions. Raises ValueError if target is unreachable.
        """
    def edgeCount(self) -> int:
        """
        Number of edges in the graph
        """
    def getNeighborsOf(self, node_id: int) -> set[int]:
        """
        IDs of the nodes connected to the given one by an edge (in any direction)
        """
    def getNodeDistances(self, source: int) -> dict[int, float]:
        """
        Topological distance (number of edges) from source to every reachable node
        """
    def getNodeIDs(self) -> list[int]:
        """
        Return a list of all node IDs
        """
    def getNodePose(self, node_id: int) -> mrpt.poses.CPose3D:
        """
        Get the estimated pose for a node
        """
    def hasNode(self, node_id: int) -> bool:
        """
        True if a node with the given ID exists
        """
    def insertEdge(self, from_id: int, to_id: int, edge: mrpt.poses.CPose3D) -> None:
        """
        Insert a directed edge from → to
        """
    def loadFromTextFile(self, fileName: str, collapse_dup_edges: bool = True) -> None:
        """
        Loads the graph from a text file in the TORO / g2o format.
        """
    def nodeCount(self) -> int:
        """
        Number of nodes in the graph
        """
    def saveToTextFile(self, fileName: str) -> None:
        """
        Saves the graph to a text file in the TORO / g2o format.
        """
    def setNodePose(self, node_id: int, pose: mrpt.poses.CPose3D) -> None:
        """
        Set the estimated pose for a node
        """
    @property
    def root(self) -> int:
        """
        Root node ID (default: 0)
        """
    @root.setter
    def root(self, arg0: int) -> None:
        ...
