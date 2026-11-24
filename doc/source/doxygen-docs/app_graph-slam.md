\page app_graph-slam graph-slam

# graph-slam

Command-line tool to inspect, optimize and visualize pose graphs (2D or 3D)
stored as text files in the common `.graph` format (`VERTEX2`, `EDGE2`,
`VERTEX3`, `EDGE3` lines, as used by TORO, and the 2D g2o `VERTEX_SE2`,
`EDGE_SE2` lines). Graphs are loaded into mrpt::graphs::CNetworkOfPoses2D or
mrpt::graphs::CNetworkOfPoses3D, and optimized with the sparse
Levenberg-Marquardt method of mrpt::graphslam::optimize_graph_spa_levmarq().

## Usage

    graph-slam -i <input.graph> (--2d | --3d) <operation> [options]

Exactly one of `--2d` or `--3d` must be given. Operations (exactly one, but
`--view` can be combined with the others):

| Option | Description |
|---|---|
| `--info` | Load the graph and print the number of nodes and edges. |
| `--dijkstra` | Estimate the global pose of all nodes from a spanning tree (Dijkstra) of the relative poses in the edges (CNetworkOfPoses::dijkstra_nodes_estimate()). |
| `--levmarq` | Optimize the graph with sparse Levenberg-Marquardt. |
| `--view` | Show the graph in a 3D window. Alone, it shows the input graph; with another operation, it shows the resulting graph. |

Other options:

| Option | Description |
|---|---|
| `-i`, `--input FILE` | Input graph file (required). |
| `-o`, `--output FILE` | Save the resulting graph to this file. |
| `--max-iters N` | Maximum number of iterations (`--levmarq`). |
| `--initial-lambda X` | Initial lambda parameter (`--levmarq`). |
| `--no-span` | Don't initialize the node poses from a Dijkstra spanning tree before optimizing (`--levmarq`). |
| `-q`, `--quiet` | Terse output. |
| `-h`, `--help`, `--version` | Show help or version. |

## Examples

    graph-slam --2d --info -i modules/mrpt_data/datasets/graph_2d_circle_50nodes.graph
    graph-slam --2d --levmarq -i modules/mrpt_data/datasets/graph_2d_circle_50nodes.graph -o out.graph
    graph-slam --2d --levmarq --view -i modules/mrpt_data/datasets/graph_2d_circle_50nodes.graph

## 3D viewer

The `--view` window shows the nodes, their coordinate frames and the edges.
Keys: `h` prints this list to the console; `i`, `e`, `c`, `v` toggle node IDs,
edges, node frames and edge relative poses; `+`/`-` change the size of node
points, `o`/`p` and `k`/`l` the size of node and edge frames; `s` saves the 3D scene to
`dump_graph_NNNNN.3Dscene`; `q` quits.

## See also

- [Graph SLAM example](page_graphslam_example.html)
- mrpt::graphs::CNetworkOfPoses, mrpt::graphslam::optimize_graph_spa_levmarq()
