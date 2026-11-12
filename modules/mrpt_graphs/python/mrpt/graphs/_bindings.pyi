"""
Python bindings for mrpt::graphs: pose graph and graph algorithms
"""
from __future__ import annotations
import mrpt.poses
__all__: list[str] = ['CNetworkOfPoses2D', 'CNetworkOfPoses2DCov', 'CNetworkOfPoses2DInf', 'CNetworkOfPoses3D', 'CNetworkOfPoses3DCov', 'CNetworkOfPoses3DInf']
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
    def clearEdges(self) -> None:
        """
        Erases all the edges.
        """
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
    def edgeExists(self, from_id: int, to_id: int) -> bool:
        """
        Returns true if the directed edge from_id -> to_id exists.
        """
    def eraseEdge(self, from_id: int, to_id: int) -> None:
        """
        Erases all the edges from_id -> to_id (nothing happens if none exists).
        """
    def getEdge(self, from_id: int, to_id: int) -> mrpt.poses.CPose2D:
        """
        Returns a copy of the edge from_id -> to_id (the first one, if there are several). Raises KeyError if it does not exist.
        """
    def getEdges(self) -> list[tuple]:
        """
        Returns all the edges as a list of tuples (from_id, to_id, edge), sorted by node IDs.
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
    def clearEdges(self) -> None:
        """
        Erases all the edges.
        """
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
    def edgeExists(self, from_id: int, to_id: int) -> bool:
        """
        Returns true if the directed edge from_id -> to_id exists.
        """
    def eraseEdge(self, from_id: int, to_id: int) -> None:
        """
        Erases all the edges from_id -> to_id (nothing happens if none exists).
        """
    def getEdge(self, from_id: int, to_id: int) -> mrpt.poses.CPose3D:
        """
        Returns a copy of the edge from_id -> to_id (the first one, if there are several). Raises KeyError if it does not exist.
        """
    def getEdges(self) -> list[tuple]:
        """
        Returns all the edges as a list of tuples (from_id, to_id, edge), sorted by node IDs.
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
class CNetworkOfPoses2DCov:
    """
    A graph of 2D poses (nodes, CPose2D) and relative pose constraints with a covariance matrix (edges, CPosePDFGaussian).
    """
    def __init__(self) -> None:
        """
        Creates an empty graph.
        """
    def __repr__(self) -> str:
        ...
    def clearEdges(self) -> None:
        """
        Erases all the edges.
        """
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
    def edgeExists(self, from_id: int, to_id: int) -> bool:
        """
        Returns true if the directed edge from_id -> to_id exists.
        """
    def eraseEdge(self, from_id: int, to_id: int) -> None:
        """
        Erases all the edges from_id -> to_id (nothing happens if none exists).
        """
    def getEdge(self, from_id: int, to_id: int) -> mrpt.poses.CPosePDFGaussian:
        """
        Returns a copy of the edge from_id -> to_id (the first one, if there are several). Raises KeyError if it does not exist.
        """
    def getEdges(self) -> list[tuple]:
        """
        Returns all the edges as a list of tuples (from_id, to_id, edge), sorted by node IDs.
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
    def insertEdge(self, from_id: int, to_id: int, edge: mrpt.poses.CPosePDFGaussian) -> None:
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
class CNetworkOfPoses3DCov:
    """
    A graph of 3D poses (nodes, CPose3D) and relative pose constraints with a covariance matrix (edges, CPose3DPDFGaussian).
    """
    def __init__(self) -> None:
        """
        Creates an empty graph.
        """
    def __repr__(self) -> str:
        ...
    def clearEdges(self) -> None:
        """
        Erases all the edges.
        """
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
    def edgeExists(self, from_id: int, to_id: int) -> bool:
        """
        Returns true if the directed edge from_id -> to_id exists.
        """
    def eraseEdge(self, from_id: int, to_id: int) -> None:
        """
        Erases all the edges from_id -> to_id (nothing happens if none exists).
        """
    def getEdge(self, from_id: int, to_id: int) -> mrpt.poses.CPose3DPDFGaussian:
        """
        Returns a copy of the edge from_id -> to_id (the first one, if there are several). Raises KeyError if it does not exist.
        """
    def getEdges(self) -> list[tuple]:
        """
        Returns all the edges as a list of tuples (from_id, to_id, edge), sorted by node IDs.
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
    def insertEdge(self, from_id: int, to_id: int, edge: mrpt.poses.CPose3DPDFGaussian) -> None:
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
class CNetworkOfPoses2DInf:
    """
    A graph of 2D poses (nodes, CPose2D) and relative pose constraints with an information matrix (edges, CPosePDFGaussianInf).
    """
    def __init__(self) -> None:
        """
        Creates an empty graph.
        """
    def __repr__(self) -> str:
        ...
    def clearEdges(self) -> None:
        """
        Erases all the edges.
        """
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
    def edgeExists(self, from_id: int, to_id: int) -> bool:
        """
        Returns true if the directed edge from_id -> to_id exists.
        """
    def eraseEdge(self, from_id: int, to_id: int) -> None:
        """
        Erases all the edges from_id -> to_id (nothing happens if none exists).
        """
    def getEdge(self, from_id: int, to_id: int) -> mrpt.poses.CPosePDFGaussianInf:
        """
        Returns a copy of the edge from_id -> to_id (the first one, if there are several). Raises KeyError if it does not exist.
        """
    def getEdges(self) -> list[tuple]:
        """
        Returns all the edges as a list of tuples (from_id, to_id, edge), sorted by node IDs.
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
    def insertEdge(self, from_id: int, to_id: int, edge: mrpt.poses.CPosePDFGaussianInf) -> None:
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
class CNetworkOfPoses3DInf:
    """
    A graph of 3D poses (nodes, CPose3D) and relative pose constraints with an information matrix (edges, CPose3DPDFGaussianInf).
    """
    def __init__(self) -> None:
        """
        Creates an empty graph.
        """
    def __repr__(self) -> str:
        ...
    def clearEdges(self) -> None:
        """
        Erases all the edges.
        """
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
    def edgeExists(self, from_id: int, to_id: int) -> bool:
        """
        Returns true if the directed edge from_id -> to_id exists.
        """
    def eraseEdge(self, from_id: int, to_id: int) -> None:
        """
        Erases all the edges from_id -> to_id (nothing happens if none exists).
        """
    def getEdge(self, from_id: int, to_id: int) -> mrpt.poses.CPose3DPDFGaussianInf:
        """
        Returns a copy of the edge from_id -> to_id (the first one, if there are several). Raises KeyError if it does not exist.
        """
    def getEdges(self) -> list[tuple]:
        """
        Returns all the edges as a list of tuples (from_id, to_id, edge), sorted by node IDs.
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
    def insertEdge(self, from_id: int, to_id: int, edge: mrpt.poses.CPose3DPDFGaussianInf) -> None:
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
