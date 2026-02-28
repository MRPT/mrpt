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
#pragma once

#include <mrpt/graphs/TNodeID.h>

#include <sstream>
#include <stdexcept>
#include <string>

namespace mrpt::graphs
{
/** Exception thrown when a THypothesis cannot be found. */
struct HypothesisNotFoundException : public std::runtime_error
{
  HypothesisNotFoundException(TNodeID from, TNodeID to) :
      std::runtime_error(
          std::string("Hypothesis not found between nodes ") + std::to_string(from) + " -> " +
          std::to_string(to))
  {
  }
  explicit HypothesisNotFoundException(size_t id) :
      std::runtime_error(std::string("Hypothesis not found with ID ") + std::to_string(id))
  {
  }
};

namespace detail
{
/** Represents a hypothesis (potential edge) between two nodes in a graph,
 * used by the graphslam-engine infrastructure. \ingroup mrpt_graphs_grp */
template <class GRAPH_T>
struct THypothesis
{
  using constraint_t = typename GRAPH_T::constraint_t;

  TNodeID from{0};
  TNodeID to{0};
  int id{0};
  bool is_valid{true};
  double goodness{0.0};

  void setEdge(const constraint_t& edge) { m_edge = edge; }
  const constraint_t& getEdge() const { return m_edge; }

  bool hasEnds(TNodeID a, TNodeID b) const { return (from == a && to == b); }

  std::string getAsString(bool oneline = false) const
  {
    std::ostringstream ss;
    if (oneline)
    {
      ss << "Hypot #" << id << " [" << from << "->" << to << "] valid=" << is_valid
         << " goodness=" << goodness;
    }
    else
    {
      ss << "Hypothesis #" << id << ":\n"
         << "  from: " << from << "\n"
         << "  to:   " << to << "\n"
         << "  is_valid: " << is_valid << "\n"
         << "  goodness: " << goodness << "\n";
    }
    return ss.str();
  }

 private:
  constraint_t m_edge;
};
}  // namespace detail
}  // namespace mrpt::graphs
