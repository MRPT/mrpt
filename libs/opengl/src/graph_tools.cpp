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

#include <mrpt/opengl.h>  // Precompiled header

#include <mrpt/opengl/graph_tools.h>

using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::opengl;

// ============================================
//           graph_visualize_2D
// ============================================
template<class CPOSE>
CSetOfObjectsPtr graph_tools::graph_visualize_2D(
	const CNetworkOfPoses<CPOSE> &g,
	const TParametersDouble &extra_params
	)
{
	MRPT_TRY_START

	CSetOfObjectsPtr ret = CSetOfObjects::Create();

	const bool   show_ID_labels = extra_params.getWithDefaultVal("show_ID_labels", false);
	const bool   show_ground_grid = extra_params.getWithDefaultVal("show_ground_grid", true);
	const bool   show_edges = extra_params.getWithDefaultVal("show_edges", true);
	const bool   show_node_corners = extra_params.getWithDefaultVal("show_node_corners", true);
	const double nodes_point_size = extra_params.getWithDefaultVal("nodes_point_size", 0. );
	const unsigned int nodes_point_color = extra_params.getWithDefaultVal("nodes_point_color", (unsigned int)0xA0A0A0 );


	if (show_ground_grid)
	{
		// Estimate bounding box.
		TPoint2D  BB_min(-10.,-10.), BB_max(10.,10.);

		for (typename CNetworkOfPoses<CPOSE>::global_poses_t::const_iterator itNod = g.nodes.begin();itNod!=g.nodes.end();++itNod)
		{
			const CPose2D p = CPose2D(itNod->second); // Convert to 2D from whatever its real type.

			keep_min( BB_min.x, p.x() );
			keep_min( BB_min.y, p.y() );
			keep_max( BB_max.x, p.x() );
			keep_max( BB_max.y, p.y() );
		}

		// Create ground plane:
		const double grid_frequency = 5.0;
		CGridPlaneXYPtr grid = CGridPlaneXY::Create(BB_min.x, BB_max.x, BB_min.y, BB_max.y, 0 /* z */, grid_frequency);
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
		for (typename CNetworkOfPoses<CPOSE>::global_poses_t::const_iterator itNod = g.nodes.begin();itNod!=g.nodes.end();++itNod)
		{
			const CPose2D p = CPose2D(itNod->second); // Convert to 2D from whatever its real type.
			pnts->insertPoint(p.x(),p.y(), 0 );
		}

		pnts->enablePointSmooth();

		ret->insert(pnts);
	} // end draw node points

	// Show a 2D corner at each node (or just an empty object with the ID label)
	if (show_node_corners || show_ID_labels)
	{
		for (typename CNetworkOfPoses<CPOSE>::global_poses_t::const_iterator itNod = g.nodes.begin();itNod!=g.nodes.end();++itNod)
		{
			const CPose2D p = CPose2D(itNod->second); // Convert to 2D from whatever its real type.

			CSetOfObjectsPtr gl_corner = show_node_corners ?
				stock_objects::CornerXYSimple(0.5 /*scale*/, 1.0 /*line width*/ ) : CSetOfObjects::Create();
			gl_corner->setPose( CPose3D(p) );
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
		gl_edges->setColor( TColorf(0,0,1, 0.3) ); XXX
		gl_edges->setLineWidth( 2.0 );

		for (typename CNetworkOfPoses<CPOSE>::const_iterator itEd = g.begin();itEd!=g.end();++itEd)
		{
			const TNodeID id1 = itEd->first.first;
			const TNodeID id2 = itEd->first.second;

			// Draw only if we have the global coords of both nodes:
			typename CNetworkOfPoses<CPOSE>::global_poses_t::const_iterator itNod1 = g.nodes.find(id1);
			typename CNetworkOfPoses<CPOSE>::global_poses_t::const_iterator itNod2 = g.nodes.find(id2);
			if (itNod1!=g.nodes.end() && itNod2!=g.nodes.end())
			{
				const CPose2D p1 = CPose2D(itNod1->second);
				const CPose2D p2 = CPose2D(itNod2->second);
				gl_edges->appendLine( TPoint3D(p1.x(),p1.y(),0), TPoint3D(p2.x(),p2.y(),0) );
			}
		}
		ret->insert( gl_edges );

	} // end draw edges

	return ret;

	MRPT_TRY_END
}

// Explicit instantiations:
template CSetOfObjectsPtr OPENGL_IMPEXP mrpt::opengl::graph_tools::graph_visualize_2D<CPosePDFGaussian>(const CNetworkOfPoses<CPosePDFGaussian> &g,const TParametersDouble &extra_params );
template CSetOfObjectsPtr OPENGL_IMPEXP mrpt::opengl::graph_tools::graph_visualize_2D<CPose3DPDFGaussian>(const CNetworkOfPoses<CPose3DPDFGaussian> &g,const TParametersDouble &extra_params );
template CSetOfObjectsPtr OPENGL_IMPEXP mrpt::opengl::graph_tools::graph_visualize_2D<CPosePDFGaussianInf>(const CNetworkOfPoses<CPosePDFGaussianInf> &g,const TParametersDouble &extra_params );
template CSetOfObjectsPtr OPENGL_IMPEXP mrpt::opengl::graph_tools::graph_visualize_2D<CPose3DPDFGaussianInf>(const CNetworkOfPoses<CPose3DPDFGaussianInf> &g,const TParametersDouble &extra_params );

