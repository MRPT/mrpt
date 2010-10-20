/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */
#ifndef opengl_graph_tools_H
#define opengl_graph_tools_H

#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/poses/CNetworkOfPoses.h>

namespace mrpt
{
	namespace opengl
	{
		/** Tool functions for graphs of pose constraints.  */
		namespace graph_tools
		{
			/** @name Tool functions for graphs of pose constraints
			    @{ */

			/** Returns an opengl objects representation of an arbitrary graph, as a network of 3D pose frames.
			  *  Note that the "global" coordinates of each node are taken from mrpt::poses::CNetworkOfPoses::nodes, so
			  *   if a node appears in "edges" but not in "nodes" it will be not displayed.
			  *
			  *  \param g             The graph
			  *  \param extra_params  An extra set of optional parameters (see below).
			  * List of accepted extra parameters (note that all are double values, booleans are emulated with 0 & !=0 values):
			  *
			  *   <table align="center" >
			  *   <tr> <td align="center" ><b>Parameter name</b></td> <td align="center" > <b>Description</b> </td> <td align="center" ><b>Default value</b></td> </tr>
			  *   <tr>
			  *        <td align="center" ><code> show_ID_labels </code></td>
			  *        <td> If set to !=0, show poses ID labels </td>
			  *        <td align="center" > 0 (don't show) </td>
			  *   </tr>
			  *   <tr>
			  *        <td align="center" ><code> show_ground_grid </code></td>
			  *        <td> If set to !=0, create a gray grid on the ground level (mrpt::opengl::CGridPlaneXY). The extension of the grid is computed to cover the entire graph extension </td>
			  *        <td align="center" > 1 (do show) </td>
			  *   </tr>
			  *   <tr>
			  *        <td align="center" ><code> show_edges </code></td>
			  *        <td> If set to !=0, draw lines between nodes with at least one edge between them. </td>
			  *        <td align="center" > 1 (do show) </td>
			  *   </tr>
			  *   <tr>
			  *        <td align="center" ><code> edge_color </code></td>
			  *        <td> If show_edges is !=0, the color of those edges as a hexadecimal int value 0xAARRGGBB with Alpha+RGB color (Alpha=0xFF:opaque,0x00:transparent).</td>
			  *        <td align="center" > 0x400000FF </td>
			  *   </tr>
			  *   <tr>
			  *        <td align="center" ><code> edge_width </code></td>
			  *        <td> If show_edges is !=0, the width of edge lines.</td>
			  *        <td align="center" > 2.0 </td>
			  *   </tr>
			  *   <tr>
			  *        <td align="center" ><code> show_node_corners </code></td>
			  *        <td> If set to !=0, draw a small 3D corner frame at each node (see mrpt::opengl::stock_objects::CornerXYZSimple). </td>
			  *        <td align="center" > 1 (do show) </td>
			  *   </tr>
			  *   <tr>
			  *        <td align="center" ><code> nodes_corner_scale </code></td>
			  *        <td> If show_node_corners!=0, the size (length) of te corner lines. </td>
			  *        <td align="center" > 0.7 </td>
			  *   </tr>
			  *   <tr>
			  *        <td align="center" ><code> nodes_point_size </code></td>
			  *        <td> If set to !=0, draw a point of the given size (glPointSize) at each node.</td>
			  *        <td align="center" > 0 (no points) </td>
			  *   </tr>
			  *   <tr>
			  *        <td align="center" ><code> nodes_point_color </code></td>
			  *        <td> If nodes_point_size!=0, set this value to a hexadecimal int value 0xRRGGBB with the desired RGB color of points.</td>
			  *        <td align="center" > 0xA0A0A0 (light gray) </td>
			  *   </tr>
			  *	  </table>
			  *
			  * \sa mrpt::poses::CNetworkOfPoses2D, mrpt::poses::CNetworkOfPoses3D, mrpt::poses::CNetworkOfPoses2DInf, mrpt::poses::CNetworkOfPoses3DInf
			  */
			template<class CPOSE,class MAPIMPL>
			CSetOfObjectsPtr OPENGL_IMPEXP graph_visualize(
				const mrpt::poses::CNetworkOfPoses<CPOSE,MAPIMPL> &g,
				const mrpt::utils::TParametersDouble &extra_params = mrpt::utils::TParametersDouble()
				);

			/** @} */
		}
	}

} // End of namespace


#endif
