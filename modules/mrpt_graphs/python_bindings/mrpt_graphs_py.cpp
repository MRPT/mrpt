/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/graphs/TNodeID.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace pybind11::literals;

// Helper: expose CNetworkOfPoses<CPOSE> as the given Python class name
template <typename CPOSE>
void bind_CNetworkOfPoses(py::module& m, const char* className)
{
  using G = mrpt::graphs::CNetworkOfPoses<CPOSE>;
  using EdgeMap = typename mrpt::graphs::CDirectedGraph<CPOSE>::edges_map_t;

  py::class_<G>(m, className)
      .def(py::init<>())
      // Node/edge counts
      .def("edgeCount", &G::edgeCount, "Number of edges in the graph")
      .def(
          "nodeCount", [](const G& g) { return g.nodes.size(); }, "Number of nodes in the graph")
      // Insert edge
      .def(
          "insertEdge",
          [](G& g, mrpt::graphs::TNodeID from, mrpt::graphs::TNodeID to, const CPOSE& edge_value)
          { g.insertEdge(from, to, edge_value); },
          "from_id"_a, "to_id"_a, "edge"_a, "Insert a directed edge from → to")
      // Node poses
      .def(
          "setNodePose",
          [](G& g, mrpt::graphs::TNodeID id, const CPOSE& pose) { g.nodes[id] = pose; },
          "node_id"_a, "pose"_a, "Set the estimated pose for a node")
      .def(
          "getNodePose", [](const G& g, mrpt::graphs::TNodeID id) { return g.nodes.at(id); },
          "node_id"_a, "Get the estimated pose for a node")
      .def(
          "hasNode", [](const G& g, mrpt::graphs::TNodeID id) { return g.nodes.count(id) > 0; },
          "node_id"_a, "True if a node with the given ID exists")
      // Get all node IDs
      .def(
          "getNodeIDs",
          [](const G& g)
          {
            std::vector<mrpt::graphs::TNodeID> ids;
            for (const auto& kv : g.nodes) ids.push_back(kv.first);
            return ids;
          },
          "Return a list of all node IDs")
      // root
      .def_readwrite("root", &G::root, "Root node ID (default: 0)")
      // File I/O
      .def("saveToTextFile", &G::saveToTextFile, "fileName"_a)
      .def(
          "loadFromTextFile",
          [](G& g, const std::string& fn, bool collapse_dup)
          { g.loadFromTextFile(fn, collapse_dup); },
          "fileName"_a, "collapse_dup_edges"_a = true)
      .def(
          "__repr__",
          [className](const G& g)
          {
            return std::string(className) + "(nodes=" + std::to_string(g.nodes.size()) +
                   ", edges=" + std::to_string(g.edgeCount()) + ")";
          });
}

PYBIND11_MODULE(_bindings, m)
{
  m.doc() = "Python bindings for mrpt::graphs — pose graph and graph algorithms";

  // -------------------------------------------------------------------------
  // CNetworkOfPoses2D — 2D pose graph with CPose2D edges
  // -------------------------------------------------------------------------
  bind_CNetworkOfPoses<mrpt::poses::CPose2D>(m, "CNetworkOfPoses2D");

  // -------------------------------------------------------------------------
  // CNetworkOfPoses3D — 3D pose graph with CPose3D edges
  // -------------------------------------------------------------------------
  bind_CNetworkOfPoses<mrpt::poses::CPose3D>(m, "CNetworkOfPoses3D");
}
