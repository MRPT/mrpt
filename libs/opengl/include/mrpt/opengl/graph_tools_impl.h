/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef opengl_graph_tools_impl_H
#define opengl_graph_tools_impl_H

#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CSimpleLine.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/stock_objects.h>

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

				using mrpt::poses::CPose3D;
				using mrpt::math::TPose3D;
				using namespace mrpt::utils;

				// Is a 2D or 3D graph network?
				typedef typename GRAPH_T::constraint_t constraint_t;

				const bool is_3D_graph = constraint_t::is_3D();

				CSetOfObjectsPtr ret = CSetOfObjects::Create();

				const bool   show_ID_labels           = 0!=extra_params.getWithDefaultVal("show_ID_labels", 0);
				const bool   show_ground_grid         = 0!=extra_params.getWithDefaultVal("show_ground_grid", 1);
				const bool   show_edges               = 0!=extra_params.getWithDefaultVal("show_edges", 1);
				const bool   show_node_corners        = 0!=extra_params.getWithDefaultVal("show_node_corners", 1);
				const bool   show_edge_rel_poses      = 0!=extra_params.getWithDefaultVal("show_edge_rel_poses", 0);
				const double nodes_point_size         = extra_params.getWithDefaultVal("nodes_point_size", 0. );
				const double nodes_corner_scale       = extra_params.getWithDefaultVal("nodes_corner_scale", 0.7 );
				const double nodes_edges_corner_scale = extra_params.getWithDefaultVal("nodes_edges_corner_scale", 0.4 );
				const unsigned int nodes_point_color  = extra_params.getWithDefaultVal("nodes_point_color", (unsigned int)0xA0A0A0 );
				const unsigned int edge_color         = extra_params.getWithDefaultVal("edge_color", (unsigned int)0x400000FF );
				const unsigned int edge_rel_poses_color = extra_params.getWithDefaultVal("edge_rel_poses_color", (unsigned int)0x40FF8000 );
				const double edge_width               = extra_params.getWithDefaultVal("edge_width", 2. );

				if (show_ground_grid)
				{
					// Estimate bounding box.
				 mrpt::math::TPoint3D  BB_min(-10.,-10.,0.), BB_max(10.,10.,0.);

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
				if (nodes_point_size>0)
				{
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

				if (show_edge_rel_poses)
				{
					const TColor col8bit(edge_rel_poses_color & 0xffffff, edge_rel_poses_color >> 24);

					for (typename GRAPH_T::const_iterator itEd = g.begin();itEd!=g.end();++itEd)
					{
						// Node ID of the source pose:
						const TNodeID node_id_start = g.edges_store_inverse_poses ? itEd->first.second : itEd->first.first;

						// Draw only if we have the global coords of starting nodes:
						typename GRAPH_T::global_poses_t::const_iterator itNod = g.nodes.find(node_id_start);
						if (itNod!=g.nodes.end())
						{
							const CPose3D pSource = CPose3D(itNod->second);
							// Create a set of objects at that pose and do the rest in relative coords:
							mrpt::opengl::CSetOfObjectsPtr gl_rel_edge = mrpt::opengl::CSetOfObjects::Create();
							gl_rel_edge->setPose(pSource);

							const typename GRAPH_T::constraint_no_pdf_t & edge_pose = itEd->second.getPoseMean();
							const mrpt::poses::CPoint3D edge_pose_pt = mrpt::poses::CPoint3D(edge_pose);

							mrpt::opengl::CSetOfObjectsPtr gl_edge_corner =
								(is_3D_graph ? stock_objects::CornerXYZSimple(nodes_edges_corner_scale, 1.0 /*line width*/ ) : stock_objects::CornerXYSimple(nodes_edges_corner_scale, 1.0 /*line width*/ ));

							gl_edge_corner->setPose(edge_pose);
							gl_rel_edge->insert(gl_edge_corner);

							mrpt::opengl::CSimpleLinePtr gl_line = mrpt::opengl::CSimpleLine::Create(0,0,0, edge_pose_pt.x(), edge_pose_pt.y(), edge_pose_pt.z() );
							gl_line->setColor_u8( col8bit );
							gl_line->setLineWidth(edge_width);
							gl_rel_edge->insert(gl_line);

							ret->insert( gl_rel_edge );
						}
					}
				}

				if (show_edges)
				{
					CSetOfLinesPtr  gl_edges = CSetOfLines::Create();
					const TColor col8bit(edge_color & 0xffffff, edge_color >> 24);

					gl_edges->setColor_u8( col8bit );
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
							gl_edges->appendLine( mrpt::math::TPoint3D(p1.x(),p1.y(),p1.z()), mrpt::math::TPoint3D(p2.x(),p2.y(),p2.z()) );
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
