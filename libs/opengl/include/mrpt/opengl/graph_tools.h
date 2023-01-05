/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/containers/yaml.h>
#include <mrpt/opengl/CSetOfObjects.h>

namespace mrpt
{
/** \ingroup mrpt_opengl_grp */
namespace opengl
{
/** Tool functions for graphs of pose constraints. \ingroup mrpt_opengl_grp  */
namespace graph_tools
{
/** @name Tool functions for graphs of pose constraints
	@{ */

// clang-format off
/** Returns an opengl objects representation of an arbitrary graph, as a network
 *of 3D pose frames.
 *  Note that the "global" coordinates of each node are taken from
 *mrpt::graphs::CNetworkOfPoses::nodes, so
 *   if a node appears in "edges" but not in "nodes" it will be not displayed.
 *
 *  \param g             The graph
 *  \param extra_params  An extra set of optional parameters (see below).
 *
 * List of accepted extra parameters:
 *
 *
 * | Parameter name            | Type    | Description                                                                                                                                         | Default    |
 * |---------------------------|---------|-----------------------------------------------------------------------------------------------------------------------------------------------------|------------|
 * | show_ID_labels            | bool    | Show poses ID labels                                                                                                                                | false      |
 * | show_ground_grid          | bool    | Creates a gray grid on the ground level (mrpt::opengl::CGridPlaneXY). The extension of the grid is computed to cover the entire graph extension     | true       |
 * | show_edges                | bool    | Draw lines between nodes with at least one edge between them.                                                                                       | true       |
 * | show_node_corners         | bool    | Draw a small 3D corner frame at each node (see mrpt::opengl::stock_objects::CornerXYZSimple)                                                        | true       |
 * | show_edge_rel_poses       | bool    | Draw the relative poses stored in each edge as a small extra 3D corner frame at each "node pose \oplus edge pose"                                   | false      |
 * | nodes_point_size          | double  | If set to !=0, draw a point of the given size (glPointSize) at each node.                                                                           | 0.0        |
 * | nodes_corner_scale        | double  | If show_node_corners==true, the size (length) of the corner lines.                                                                                  | 0.7        |
 * | nodes_edges_corner_scale  | double  | If show_edge_rel_poses==true, the size of the corners at the end of each drawn edge                                                                 | 0.4        |
 * | nodes_point_color         | int     | If nodes_point_size>0, set this value to a hexadecimal int value 0xRRGGBBAA with the desired RGB+Alpha color of points.                             | 0xA0A0A0FF |
 * | edge_color                | int     | If show_edges==true, the color of those edges as a hexadecimal int value 0xRRGGBBAA with RGB+Alpha color.                                           | 0x0000FF40 |
 * | edge_rel_poses_color      | int     | If show_edge_rel_poses==true, the color of those edges as a hexadecimal int value 0xRRGGBBAA with RGB+Alpha color.                                  | 0xFF800040 |
 * | edge_width                | double  | If show_edges==true, the width of edge lines.                                                                                                       | 1          |
 *
 * \sa mrpt::graphs::CNetworkOfPoses2D, mrpt::graphs::CNetworkOfPoses3D,
 *mrpt::graphs::CNetworkOfPoses2DInf, mrpt::graphs::CNetworkOfPoses3DInf
 * \note Implemented as headers-only in \a graph_tools_impl.h
 * \ingroup mrpt_opengl_grp
 */
// clang-format on
template <class GRAPH_T>
CSetOfObjects::Ptr graph_visualize(
	const GRAPH_T& g, const mrpt::containers::yaml& extra_params = {});

/** @} */
}  // namespace graph_tools
}  // namespace opengl

}  // namespace mrpt

#include "graph_tools_impl.h"
