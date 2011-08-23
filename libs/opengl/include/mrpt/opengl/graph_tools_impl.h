/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
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
#ifndef opengl_graph_tools_impl_H
#define opengl_graph_tools_impl_H

namespace mrpt
{
	namespace opengl
	{
		namespace graph_tools
		{
			template<class GRAPH_T>
			CSetOfObjectsPtr graph_visualize(
				const GRAPH_T &g,
				const mrpt::utils::TParametersDouble &extra_params)
			{
				MRPT_TRY_START

				// Is a 2D or 3D graph network?
				typedef typename GRAPH_T::constraint_t constraint_t;

				const bool is_3D_graph = constraint_t::is_3D();

				CSetOfObjectsPtr ret = CSetOfObjects::Create();

				const bool   show_ID_labels           = 0!=extra_params.getWithDefaultVal("show_ID_labels", 0);
				const bool   show_ground_grid         = 0!=extra_params.getWithDefaultVal("show_ground_grid", 1);
				const bool   show_edges               = 0!=extra_params.getWithDefaultVal("show_edges", 1);
				const bool   show_node_corners        = 0!=extra_params.getWithDefaultVal("show_node_corners", 1);
				const double nodes_point_size         = extra_params.getWithDefaultVal("nodes_point_size", 0. );
				const double nodes_corner_scale       = extra_params.getWithDefaultVal("nodes_corner_scale", 0.7 );
				const unsigned int nodes_point_color  = extra_params.getWithDefaultVal("nodes_point_color", (unsigned int)0xA0A0A0 );
				const unsigned int edge_color         = extra_params.getWithDefaultVal("edge_color", (unsigned int)0x400000FF );
				const double edge_width               = extra_params.getWithDefaultVal("edge_width", 2. );

				if (show_ground_grid)
				{
					// Estimate bounding box.
					TPoint3D  BB_min(-10.,-10.,0.), BB_max(10.,10.,0.);

					for (typename GRAPH_T::global_poses_t::const_iterator itNod = g.nodes.begin();itNod!=g.nodes.end();++itNod)
					{
						const CPose3D p = CPose3D(itNod->second); // Convert to 3D from whatever its real type.

						keep_min( BB_min.x, p.x() );
						keep_min( BB_min.y, p.y() );
						keep_min( BB_min.z, p.z() );

						keep_max( BB_max.x, p.x() );
						keep_max( BB_max.y, p.y() );
						keep_max( BB_max.z, p.z() );
					}

					// Create ground plane:
					const double grid_frequency = 5.0;
					CGridPlaneXYPtr grid = CGridPlaneXY::Create(BB_min.x, BB_max.x, BB_min.y, BB_max.y, BB_min.z, grid_frequency);
					grid->setColor(0.3,0.3,0.3);
					ret->insert( grid );
				} // end show_ground_grid

				// Draw nodes as thick points:
				if (nodes_point_size!=0)
				{
					ASSERT_(nodes_point_size>=0);

					CPointCloudPtr pnts = CPointCloud::Create();
					pnts->setColor( TColorf(TColor(nodes_point_color)) );
					pnts->setPointSize(nodes_point_size);

					// Add nodes:
					for (typename GRAPH_T::global_poses_t::const_iterator itNod = g.nodes.begin();itNod!=g.nodes.end();++itNod)
					{
						const CPose3D p = CPose3D(itNod->second); // Convert to 3D from whatever its real type.
						pnts->insertPoint(p.x(),p.y(), p.z() );
					}

					pnts->enablePointSmooth();

					ret->insert(pnts);
				} // end draw node points

				// Show a 2D corner at each node (or just an empty object with the ID label)
				if (show_node_corners || show_ID_labels)
				{
					for (typename GRAPH_T::global_poses_t::const_iterator itNod = g.nodes.begin();itNod!=g.nodes.end();++itNod)
					{
						const CPose3D p = CPose3D(itNod->second); // Convert to 3D from whatever its real type.
						CSetOfObjectsPtr gl_corner = show_node_corners ?
							(is_3D_graph ? stock_objects::CornerXYZSimple(nodes_corner_scale, 1.0 /*line width*/ ) : stock_objects::CornerXYSimple(nodes_corner_scale, 1.0 /*line width*/ ))
							: CSetOfObjects::Create();
						gl_corner->setPose( p );
						if (show_ID_labels) // don't show IDs twice!
						{
							gl_corner->setName(format("%u",static_cast<unsigned int>(itNod->first) ));
							gl_corner->enableShowName();
						}
						ret->insert( gl_corner );
					}
				} // end draw node corners

				if (show_edges)
				{
					CSetOfLinesPtr  gl_edges = CSetOfLines::Create();
					const TColor col8bit(edge_color & 0xffffff);
					const uint8_t col_alpha = edge_color >> 24;

					gl_edges->setColor( col8bit.R*(1./255), col8bit.G*(1./255), col8bit.B*(1./255), col_alpha*(1./255) );
					gl_edges->setLineWidth( edge_width );

					for (typename GRAPH_T::const_iterator itEd = g.begin();itEd!=g.end();++itEd)
					{
						const TNodeID id1 = itEd->first.first;
						const TNodeID id2 = itEd->first.second;

						// Draw only if we have the global coords of both nodes:
						typename GRAPH_T::global_poses_t::const_iterator itNod1 = g.nodes.find(id1);
						typename GRAPH_T::global_poses_t::const_iterator itNod2 = g.nodes.find(id2);
						if (itNod1!=g.nodes.end() && itNod2!=g.nodes.end())
						{
							const CPose3D p1 = CPose3D(itNod1->second);
							const CPose3D p2 = CPose3D(itNod2->second);
							gl_edges->appendLine( TPoint3D(p1.x(),p1.y(),p1.z()), TPoint3D(p2.x(),p2.y(),p2.z()) );
						}
					}
					ret->insert( gl_edges );

				} // end draw edges

				return ret;

				MRPT_TRY_END

			}

		}
	}
}

#endif
