"""
Python bindings for mrpt::graphslam: pose-graph optimization
"""
from __future__ import annotations
import mrpt.graphs
import typing
__all__: list[str] = ['TResultInfoSpaLevMarq', 'optimize_graph_spa_levmarq']
class TResultInfoSpaLevMarq:
    """
    Output information of optimize_graph_spa_levmarq().
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def __repr__(self) -> str:
        ...
    @property
    def final_total_sq_error(self) -> float:
        """
        Sum of the squared errors of all the edges (squared Mahalanobis distances for edges with an information matrix).
        """
    @final_total_sq_error.setter
    def final_total_sq_error(self, arg0: float) -> None:
        ...
    @property
    def num_iters(self) -> int:
        """
        Index of the last Levenberg-Marquardt iteration executed.
        """
    @num_iters.setter
    def num_iters(self, arg0: int) -> None:
        ...
@typing.overload
def optimize_graph_spa_levmarq(graph: mrpt.graphs.CNetworkOfPoses2D, nodes_to_optimize: set[int] | None = None, extra_params: dict | None = None, feedback: typing.Callable | None = None) -> TResultInfoSpaLevMarq:
    """
    Optimizes the node poses of a pose graph (in place) with the Sparse Pose Adjustment (SPA) method and Levenberg-Marquardt, from the initial estimate in its nodes (e.g. from dijkstra_nodes_estimate()).
    
    nodes_to_optimize: set of node IDs to optimize (default: all but the root).
    extra_params: optional dict with any of: verbose (bool, default False), max_iterations (default 100), profiler (bool, default False), initial_lambda (default 0: automatic), tau (default 1e-3), e1 (default 1e-6: stop if |gradient| < e1), e2 (default 1e-6: stop if |delta| < e2*(|x|+e2)).
    feedback: optional function(graph, iteration, max_iterations, total_sq_error) called on each iteration.
    Returns a TResultInfoSpaLevMarq. Edges with information matrices (the *Inf graphs) weight the residuals; other edges have unit information.
    """
@typing.overload
def optimize_graph_spa_levmarq(graph: mrpt.graphs.CNetworkOfPoses3D, nodes_to_optimize: set[int] | None = None, extra_params: dict | None = None, feedback: typing.Callable | None = None) -> TResultInfoSpaLevMarq:
    """
    Optimizes the node poses of a pose graph (in place) with the Sparse Pose Adjustment (SPA) method and Levenberg-Marquardt, from the initial estimate in its nodes (e.g. from dijkstra_nodes_estimate()).
    
    nodes_to_optimize: set of node IDs to optimize (default: all but the root).
    extra_params: optional dict with any of: verbose (bool, default False), max_iterations (default 100), profiler (bool, default False), initial_lambda (default 0: automatic), tau (default 1e-3), e1 (default 1e-6: stop if |gradient| < e1), e2 (default 1e-6: stop if |delta| < e2*(|x|+e2)).
    feedback: optional function(graph, iteration, max_iterations, total_sq_error) called on each iteration.
    Returns a TResultInfoSpaLevMarq. Edges with information matrices (the *Inf graphs) weight the residuals; other edges have unit information.
    """
@typing.overload
def optimize_graph_spa_levmarq(graph: mrpt.graphs.CNetworkOfPoses2DInf, nodes_to_optimize: set[int] | None = None, extra_params: dict | None = None, feedback: typing.Callable | None = None) -> TResultInfoSpaLevMarq:
    """
    Optimizes the node poses of a pose graph (in place) with the Sparse Pose Adjustment (SPA) method and Levenberg-Marquardt, from the initial estimate in its nodes (e.g. from dijkstra_nodes_estimate()).
    
    nodes_to_optimize: set of node IDs to optimize (default: all but the root).
    extra_params: optional dict with any of: verbose (bool, default False), max_iterations (default 100), profiler (bool, default False), initial_lambda (default 0: automatic), tau (default 1e-3), e1 (default 1e-6: stop if |gradient| < e1), e2 (default 1e-6: stop if |delta| < e2*(|x|+e2)).
    feedback: optional function(graph, iteration, max_iterations, total_sq_error) called on each iteration.
    Returns a TResultInfoSpaLevMarq. Edges with information matrices (the *Inf graphs) weight the residuals; other edges have unit information.
    """
@typing.overload
def optimize_graph_spa_levmarq(graph: mrpt.graphs.CNetworkOfPoses3DInf, nodes_to_optimize: set[int] | None = None, extra_params: dict | None = None, feedback: typing.Callable | None = None) -> TResultInfoSpaLevMarq:
    """
    Optimizes the node poses of a pose graph (in place) with the Sparse Pose Adjustment (SPA) method and Levenberg-Marquardt, from the initial estimate in its nodes (e.g. from dijkstra_nodes_estimate()).
    
    nodes_to_optimize: set of node IDs to optimize (default: all but the root).
    extra_params: optional dict with any of: verbose (bool, default False), max_iterations (default 100), profiler (bool, default False), initial_lambda (default 0: automatic), tau (default 1e-3), e1 (default 1e-6: stop if |gradient| < e1), e2 (default 1e-6: stop if |delta| < e2*(|x|+e2)).
    feedback: optional function(graph, iteration, max_iterations, total_sq_error) called on each iteration.
    Returns a TResultInfoSpaLevMarq. Edges with information matrices (the *Inf graphs) weight the residuals; other edges have unit information.
    """
