\defgroup mrpt_graphslam_grp [mrpt-graphslam]

GraphSLAM algorithms

[TOC]

# Library mrpt-graphslam

This C++ library is part of MRPT and can be installed in Debian-based systems
with:

		sudo apt install libmrpt-graphslam-dev

Read also [how to import MRPT into your CMake scripts](mrpt_from_cmake.html).

Basic graph-SLAM algorithms: See the namespace mrpt::graphslam.

For an introduction to graph-slam maps refer to the summary in
https://www.mrpt.org/Graph-SLAM_maps

## Overview

In graph-SLAM, the robot trajectory is a graph whose nodes are robot poses
(keyframes) and whose edges are relative-pose measurements between them
(odometry, scan matching, loop closures). This module provides:

- A **pose-graph optimizer**: Levenberg-Marquardt on the Sparse Pose
  Adjustment formulation, for 2D and 3D graphs. Use it when you already have
  the graph, built by your own front-end.
- A complete, configurable **graph-SLAM engine**, which builds the graph from
  a dataset by means of pluggable node and edge registration deciders, and
  optimizes it. It is the core of the \ref app_graphslam-engine application.

The pose graph containers (mrpt::graphs::CNetworkOfPoses) are defined in
\ref mrpt_graphs_grp.

## Key classes and functions

- mrpt::graphslam::optimize_graph_spa_levmarq(): optimizes all graph types
  derived from mrpt::graphs::CNetworkOfPoses; the root node stays fixed.
- mrpt::graphslam::CGraphSlamEngine: the graph-SLAM engine, configured from
  an INI file.
- mrpt::graphslam::deciders::CICPCriteriaNRD,
  mrpt::graphslam::deciders::CLoopCloserERD,
  mrpt::graphslam::optimizers::CLevMarqGSO: examples of node registration
  deciders (NRD), edge registration deciders (ERD) and optimizers (GSO) that
  the engine combines.

## Example: optimizing a pose graph

```cpp
#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/graphslam/levmarq.h>

using mrpt::poses::CPose2D;

// Pose graph: edges are relative poses with an information matrix.
mrpt::graphs::CNetworkOfPoses2DInf graph;
graph.root = 0;  // this node stays fixed

// Initial guesses of the global poses (e.g. from odometry):
graph.nodes[0] = CPose2D(0.0, 0.0, 0.0);
graph.nodes[1] = CPose2D(1.1, 0.1, 0.0);
graph.nodes[2] = CPose2D(2.2, -0.1, 0.05);

mrpt::math::CMatrixDouble33 info;
info.setDiagonal(100.0);  // inverse covariance of each edge
graph.insertEdge(0, 1, mrpt::poses::CPosePDFGaussianInf(CPose2D(1.0, 0.0, 0.0), info));
graph.insertEdge(1, 2, mrpt::poses::CPosePDFGaussianInf(CPose2D(1.0, 0.0, 0.0), info));
graph.insertEdge(0, 2, mrpt::poses::CPosePDFGaussianInf(CPose2D(2.0, 0.0, 0.0), info));

mrpt::graphslam::TResultInfoSpaLevMarq result;
mrpt::graphslam::optimize_graph_spa_levmarq(
    graph, result, nullptr, mrpt::containers::yaml::Map({{"max_iterations", 100}}));
// graph.nodes now holds the optimized poses.
```

## Examples

- \ref graphslam_example: optimization of noisy 2D and 3D pose graphs.
- Python: [mrpt_graphslam_example.py](pymrpt_example_mrpt_graphslam_example.html).
- Applications: \ref app_graph-slam (optimizes graph files) and
  \ref app_graphslam-engine.

# Library contents
