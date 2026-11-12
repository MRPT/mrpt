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

#include <mrpt/containers/yaml.h>
#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/graphslam/levmarq.h>
#include <mrpt/graphslam/types.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <optional>
#include <set>
#include <string>

namespace py = pybind11;
using namespace pybind11::literals;

namespace
{
using mrpt::graphs::TNodeID;
using mrpt::graphslam::TResultInfoSpaLevMarq;

// The parameters read by optimize_graph_spa_levmarq():
const std::set<std::string> LEVMARQ_PARAMS = {
    "verbose", "max_iterations", "profiler", "initial_lambda", "tau", "e1", "e2"};

mrpt::containers::yaml levmarqParamsFromDict(const std::optional<py::dict>& params)
{
  auto y = mrpt::containers::yaml(mrpt::containers::yaml::Map());
  if (!params)
  {
    return y;
  }
  for (const auto& [k, v] : *params)
  {
    const auto key = py::cast<std::string>(k);
    if (LEVMARQ_PARAMS.count(key) == 0)
    {
      throw py::key_error("unknown parameter '" + key + "'");
    }
    // bool is a subclass of int in Python: check it first.
    if (py::isinstance<py::bool_>(v))
    {
      y[key] = py::cast<bool>(v);
    }
    else if (py::isinstance<py::int_>(v))
    {
      y[key] = py::cast<int64_t>(v);
    }
    else if (py::isinstance<py::float_>(v))
    {
      y[key] = py::cast<double>(v);
    }
    else
    {
      throw py::type_error("parameter '" + key + "' must be a bool, int or float");
    }
  }
  return y;
}

template <class GRAPH_T>
TResultInfoSpaLevMarq optimizeGraph(
    GRAPH_T& graph,
    const std::optional<std::set<TNodeID>>& nodesToOptimize,
    const std::optional<py::dict>& extraParams,
    const std::optional<py::function>& feedback)
{
  using feedback_t = typename mrpt::graphslam::graphslam_traits<GRAPH_T>::TFunctorFeedback;

  const auto params = levmarqParamsFromDict(extraParams);
  const std::set<TNodeID>* nodes = nodesToOptimize ? &nodesToOptimize.value() : nullptr;

  // The optimizer requires at least one free node (by default, all but the root):
  const bool anyFreeNode = nodes != nullptr
                               ? !nodes->empty()
                               : std::any_of(
                                     graph.nodes.begin(), graph.nodes.end(),
                                     [&graph](const auto& n) { return n.first != graph.root; });
  if (!anyFreeNode)
  {
    throw py::value_error("there are no nodes to optimize");
  }
  TResultInfoSpaLevMarq info{};
  feedback_t cb;
  if (feedback)
  {
    const py::function& fn = feedback.value();
    cb = [&fn](const GRAPH_T& g, size_t iter, size_t maxIter, double sqError)
    {
      py::gil_scoped_acquire gil;
      fn(py::cast(&g, py::return_value_policy::reference), iter, maxIter, sqError);
    };
  }
  // Other Python threads may run during the optimization:
  py::gil_scoped_release noGil;
  mrpt::graphslam::optimize_graph_spa_levmarq(graph, info, nodes, params, cb);
  return info;
}

template <class GRAPH_T>
void defOptimizer(py::module& m)
{
  m.def(
      "optimize_graph_spa_levmarq", &optimizeGraph<GRAPH_T>, "graph"_a,
      "nodes_to_optimize"_a = py::none(), "extra_params"_a = py::none(), "feedback"_a = py::none(),
      "Optimizes the node poses of a pose graph (in place) with the Sparse Pose Adjustment (SPA) "
      "method and Levenberg-Marquardt, from the initial estimate in its nodes (e.g. from "
      "dijkstra_nodes_estimate()).\n\n"
      "nodes_to_optimize: set of node IDs to optimize (default: all but the root).\n"
      "extra_params: optional dict with any of: verbose (bool, default False), max_iterations "
      "(default 100), profiler (bool, default False), initial_lambda (default 0: automatic), "
      "tau (default 1e-3), e1 (default 1e-6: stop if |gradient| < e1), e2 (default 1e-6: stop "
      "if |delta| < e2*(|x|+e2)).\n"
      "feedback: optional function(graph, iteration, max_iterations, total_sq_error) called on "
      "each iteration.\n"
      "Returns a TResultInfoSpaLevMarq. Edges with information matrices (the *Inf graphs) "
      "weight the residuals; other edges have unit information.");
}
}  // namespace

PYBIND11_MODULE(_bindings, m)
{
  m.doc() = "Python bindings for mrpt::graphslam: pose-graph optimization";

  py::class_<TResultInfoSpaLevMarq>(
      m, "TResultInfoSpaLevMarq", "Output information of optimize_graph_spa_levmarq().")
      .def(py::init<>(), "Default constructor.")
      .def_readwrite(
          "num_iters", &TResultInfoSpaLevMarq::num_iters,
          "Index of the last Levenberg-Marquardt iteration executed.")
      .def_readwrite(
          "final_total_sq_error", &TResultInfoSpaLevMarq::final_total_sq_error,
          "Sum of the squared errors of all the edges (squared Mahalanobis distances for edges "
          "with an information matrix).")
      .def(
          "__repr__",
          [](const TResultInfoSpaLevMarq& r)
          {
            return "TResultInfoSpaLevMarq(num_iters=" + std::to_string(r.num_iters) +
                   ", final_total_sq_error=" + std::to_string(r.final_total_sq_error) + ")";
          });

  defOptimizer<mrpt::graphs::CNetworkOfPoses2D>(m);
  defOptimizer<mrpt::graphs::CNetworkOfPoses3D>(m);
  defOptimizer<mrpt::graphs::CNetworkOfPoses2DInf>(m);
  defOptimizer<mrpt::graphs::CNetworkOfPoses3DInf>(m);
}
