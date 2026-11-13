#!/usr/bin/env python3
"""
Optimizes a noisy 2D pose graph with loop closures with mrpt.graphslam.

Demonstrates:
  - CNetworkOfPoses2DInf: pose graph whose edges have information matrices
  - dijkstra_nodes_estimate(): initial guess of the node poses from the edges
  - optimize_graph_spa_levmarq(): Levenberg-Marquardt optimization, with
    parameters and a per-iteration feedback function
"""

import math
import random

import numpy as np
from mrpt.graphs import CNetworkOfPoses2DInf
from mrpt.graphslam import optimize_graph_spa_levmarq
from mrpt.poses import CPose2D, CPosePDFGaussianInf

random.seed(1)

# ---------------------------------------------------------------------------
# Ground truth: a robot driving three laps of a square, 5 poses per side
# ---------------------------------------------------------------------------
N_LAPS, N_SIDES, POSES_PER_SIDE, STEP = 3, 4, 5, 1.0
gt = [CPose2D(0, 0, 0)]
for lap in range(N_LAPS):
    for side in range(N_SIDES):
        for k in range(POSES_PER_SIDE):
            turn = math.pi / 2 if k == POSES_PER_SIDE - 1 else 0.0
            gt.append(gt[-1] + CPose2D(STEP, 0, turn))
n_nodes = len(gt)
poses_per_lap = N_SIDES * POSES_PER_SIDE

# ---------------------------------------------------------------------------
# Noisy odometry edges, plus loop closures between the same place in each lap
# ---------------------------------------------------------------------------
odo_sigmas = np.array([0.05, 0.05, math.radians(2.0)])
loop_sigmas = np.array([0.02, 0.02, math.radians(0.5)])


def noisy_edge(i, j, sigmas):
    """The relative pose of node j from node i, with Gaussian noise."""
    true_rel = gt[j] - gt[i]
    noise = CPose2D(*[random.gauss(0, s) for s in sigmas])
    information = np.diag(1.0 / sigmas**2)
    return CPosePDFGaussianInf(true_rel + noise, information)


graph = CNetworkOfPoses2DInf()
for i in range(n_nodes - 1):
    graph.insertEdge(i, i + 1, noisy_edge(i, i + 1, odo_sigmas))
n_loops = 0
for i in range(n_nodes):
    for j in range(i + poses_per_lap, n_nodes, poses_per_lap):
        graph.insertEdge(i, j, noisy_edge(i, j, loop_sigmas))
        n_loops += 1

# Initial guess of the node poses: composing edges from the root (node 0),
# which accumulates the odometry drift.
graph.root = 0
graph.dijkstra_nodes_estimate()
print(f"Graph: {graph.nodeCount()} nodes, {graph.edgeCount()} edges ({n_loops} loop closures)")


def rmse_position(g):
    return math.sqrt(sum((g.getNodePose(i).x - p.x) ** 2 + (g.getNodePose(i).y - p.y) ** 2
                         for i, p in enumerate(gt)) / n_nodes)


# The error of the initial guess (max_iterations=0 does not change the graph):
info = optimize_graph_spa_levmarq(graph, extra_params={"max_iterations": 0})
print(f"Before: total squared error = {info.final_total_sq_error:.3f}, "
      f"RMSE of positions = {rmse_position(graph):.3f} m")


def on_iteration(g, iteration, max_iterations, total_sq_error):
    print(f"  iteration {iteration:2d}/{max_iterations}: total squared error = "
          f"{total_sq_error:.4f}")


info = optimize_graph_spa_levmarq(
    graph, extra_params={"max_iterations": 50}, feedback=on_iteration)
print(f"After:  total squared error = {info.final_total_sq_error:.3f}, "
      f"RMSE of positions = {rmse_position(graph):.3f} m")
print(info)
