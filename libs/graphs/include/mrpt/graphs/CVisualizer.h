/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/containers/yaml.h>
#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/viz/CGridPlaneXY.h>
#include <mrpt/viz/CPointCloud.h>
#include <mrpt/viz/CSetOfLines.h>
#include <mrpt/viz/CSetOfObjects.h>
#include <mrpt/viz/CSimpleLine.h>
#include <mrpt/viz/stock_objects.h>

namespace mrpt::graphs::detail
{
/** \brief Base class for C*Visualizer classes.
 *
 * By default provides visualization for a CNetowrkOfPoses containing nodes and
 * edges as constructed by a single agent/robot. Derived classes can inherit
 * and partially modify the class behavior
 */
template <
    class CPOSE,                // Type of edges
    class MAPS_IMPLEMENTATION,  // Use std::map<> vs. std::vector<>
    class NODE_ANNOTATIONS = mrpt::graphs::detail::TNodeAnnotations,
    class EDGE_ANNOTATIONS = mrpt::graphs::detail::edge_annotations_empty>
class CVisualizer
{
 public:
  using GRAPH_T =
      mrpt::graphs::CNetworkOfPoses<CPOSE, MAPS_IMPLEMENTATION, NODE_ANNOTATIONS, EDGE_ANNOTATIONS>;
  /**\brief Constructor
   */
  CVisualizer(const GRAPH_T& graph_in);
  /**\brief Destructor
   */
  virtual ~CVisualizer();
  /**\brief Common visualization stuff for all derived classes
   *
   * Function delegates visualization tasks to the draw* methods according to
   * the user preferences
   *
   */
  virtual void getAs3DObject(
      mrpt::viz::CSetOfObjects::Ptr& object, const mrpt::containers::yaml& viz_params) const;

 protected:
  /**\name Work-splitting methods
   * \brief Smaller functions that do add very specific parts to the visual
   * representation
   *
   * Following functions take an optional mrpt::containers::yaml instance
   * containing visualization parameters
   */
  /**\{ */
  virtual void drawGroundGrid(
      mrpt::viz::CSetOfObjects::Ptr& object,
      const mrpt::containers::yaml* viz_params = nullptr) const;
  virtual void drawNodeCorners(
      mrpt::viz::CSetOfObjects::Ptr& object,
      const mrpt::containers::yaml* viz_params = nullptr) const;
  virtual void drawNodePoints(
      mrpt::viz::CSetOfObjects::Ptr& object,
      const mrpt::containers::yaml* viz_params = nullptr) const;
  virtual void drawEdgeRelPoses(
      mrpt::viz::CSetOfObjects::Ptr& object,
      const mrpt::containers::yaml* viz_params = nullptr) const;
  virtual void drawEdges(
      mrpt::viz::CSetOfObjects::Ptr& object,
      const mrpt::containers::yaml* viz_params = nullptr) const;

  /**\} */

  bool m_is_3D_graph;
  const GRAPH_T& m_graph;
};
}  // namespace mrpt::graphs::detail
#include <mrpt/graphs/CVisualizer_impl.h>
