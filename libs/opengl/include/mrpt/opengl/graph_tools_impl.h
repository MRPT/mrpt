/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/img/TColor.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CSimpleLine.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/poses/CPoint3D.h>

namespace mrpt::opengl::graph_tools
{
template <class GRAPH_T>
CSetOfObjects::Ptr graph_visualize(
	const GRAPH_T& g, const mrpt::system::TParametersDouble& extra_params)
{
	MRPT_TRY_START

	using mrpt::math::TPose3D;
	using mrpt::poses::CPose3D;

	// Is a 2D or 3D graph network?
	using constraint_t = typename GRAPH_T::constraint_t;

	const bool is_3D_graph = constraint_t::is_3D();

	// create opengl obejct to be filled.
	CSetOfObjects::Ptr ret = std::make_shared<CSetOfObjects>();

	// graph visualization parameters
	const bool show_ID_labels =
		0 != extra_params.getWithDefaultVal("show_ID_labels", 0);
	const bool show_ground_grid =
		0 != extra_params.getWithDefaultVal("show_ground_grid", 1);
	const bool show_edges =
		0 != extra_params.getWithDefaultVal("show_edges", 1);
	const bool show_node_corners =
		0 != extra_params.getWithDefaultVal("show_node_corners", 1);
	const bool show_edge_rel_poses =
		0 != extra_params.getWithDefaultVal("show_edge_rel_poses", 0);
	const double nodes_point_size =
		extra_params.getWithDefaultVal("nodes_point_size", 0.);
	const double nodes_corner_scale =
		extra_params.getWithDefaultVal("nodes_corner_scale", 0.7);
	const double nodes_edges_corner_scale =
		extra_params.getWithDefaultVal("nodes_edges_corner_scale", 0.4);
	const unsigned int nodes_point_color = extra_params.getWithDefaultVal(
		"nodes_point_color", (unsigned int)0xA0A0A0);
	const unsigned int edge_color =
		extra_params.getWithDefaultVal("edge_color", (unsigned int)0x400000FF);
	const unsigned int edge_rel_poses_color = extra_params.getWithDefaultVal(
		"edge_rel_poses_color", (unsigned int)0x40FF8000);
	const double edge_width = extra_params.getWithDefaultVal("edge_width", 2.);

	if (show_ground_grid)
	{
		// Estimate bounding box.
		mrpt::math::TPoint3D BB_min(-10., -10., 0.), BB_max(10., 10., 0.);

		for (auto itNod = g.nodes.begin(); itNod != g.nodes.end(); ++itNod)
		{
			const CPose3D p = CPose3D(
				itNod->second);  // Convert to 3D from whatever its real type.

			keep_min(BB_min.x, p.x());
			keep_min(BB_min.y, p.y());
			keep_min(BB_min.z, p.z());

			keep_max(BB_max.x, p.x());
			keep_max(BB_max.y, p.y());
			keep_max(BB_max.z, p.z());
		}

		// Create ground plane:
		const double grid_frequency = 5.0;
		CGridPlaneXY::Ptr grid = CGridPlaneXY::Create(
			BB_min.x, BB_max.x, BB_min.y, BB_max.y, BB_min.z, grid_frequency);
		grid->setColor(0.3f, 0.3f, 0.3f);
		ret->insert(grid);
	}  // end show_ground_grid

	// Draw nodes as thick points:
	if (nodes_point_size > 0)
	{
		CPointCloud::Ptr pnts = std::make_shared<CPointCloud>();
		pnts->setColor(
			mrpt::img::TColorf(mrpt::img::TColor(nodes_point_color)));
		pnts->setPointSize(nodes_point_size);

		// Add nodes:
		for (auto itNod = g.nodes.begin(); itNod != g.nodes.end(); ++itNod)
		{
			const CPose3D p = CPose3D(
				itNod->second);  // Convert to 3D from whatever its real type.
			pnts->insertPoint(p.x(), p.y(), p.z());
		}

		pnts->enablePointSmooth();

		ret->insert(pnts);
	}  // end draw node points

	// Show a 2D corner at each node (or just an empty object with the ID label)
	if (show_node_corners || show_ID_labels)
	{
		for (auto itNod = g.nodes.begin(); itNod != g.nodes.end(); ++itNod)
		{
			const CPose3D p = CPose3D(
				itNod->second);  // Convert to 3D from whatever its real type.
			CSetOfObjects::Ptr gl_corner =
				show_node_corners
					? (is_3D_graph
						   ? stock_objects::CornerXYZSimple(
								 nodes_corner_scale, 1.0 /*line width*/)
						   : stock_objects::CornerXYSimple(
								 nodes_corner_scale, 1.0 /*line width*/))
					: std::make_shared<CSetOfObjects>();
			gl_corner->setPose(p);
			if (show_ID_labels)  // don't show IDs twice!
			{
				gl_corner->setName(
					format("%u", static_cast<unsigned int>(itNod->first)));
				gl_corner->enableShowName();
			}
			ret->insert(gl_corner);
		}
	}  // end draw node corners

	if (show_edge_rel_poses)
	{
		const mrpt::img::TColor col8bit(
			edge_rel_poses_color & 0xffffff, edge_rel_poses_color >> 24);

		for (const auto& edge : g)
		{
			// Node ID of the source pose:
			const auto node_id_start = g.edges_store_inverse_poses
										   ? edge.first.second
										   : edge.first.first;

			// Draw only if we have the global coords of starting nodes:
			auto itNod = g.nodes.find(node_id_start);
			if (itNod != g.nodes.end())
			{
				const CPose3D pSource = CPose3D(itNod->second);
				// Create a set of objects at that pose and do the rest in
				// relative coords:
				auto gl_rel_edge = mrpt::opengl::CSetOfObjects::Create();
				gl_rel_edge->setPose(pSource);

				const auto& edge_pose = edge.second.getPoseMean();
				const auto edge_pose_pt = mrpt::poses::CPoint3D(edge_pose);

				auto gl_edge_corner =
					(is_3D_graph
						 ? stock_objects::CornerXYZSimple(
							   nodes_edges_corner_scale, 1.0 /*line width*/)
						 : stock_objects::CornerXYSimple(
							   nodes_edges_corner_scale, 1.0 /*line width*/));

				gl_edge_corner->setPose(edge_pose);
				gl_rel_edge->insert(gl_edge_corner);

				auto gl_line = mrpt::opengl::CSimpleLine::Create(
					0, 0, 0, edge_pose_pt.x(), edge_pose_pt.y(),
					edge_pose_pt.z());
				gl_line->setColor_u8(col8bit);
				gl_line->setLineWidth(edge_width);
				gl_rel_edge->insert(gl_line);

				ret->insert(gl_rel_edge);
			}
		}
	}

	if (show_edges)
	{
		CSetOfLines::Ptr gl_edges = std::make_shared<CSetOfLines>();
		const mrpt::img::TColor col8bit(
			edge_color & 0xffffff, edge_color >> 24);

		gl_edges->setColor_u8(col8bit);
		gl_edges->setLineWidth(edge_width);

		for (const auto& edge : g)
		{
			const auto id1 = edge.first.first;
			const auto id2 = edge.first.second;

			// Draw only if we have the global coords of both nodes:
			auto itNod1 = g.nodes.find(id1);
			auto itNod2 = g.nodes.find(id2);
			if (itNod1 != g.nodes.end() && itNod2 != g.nodes.end())
			{
				const CPose3D p1 = CPose3D(itNod1->second);
				const CPose3D p2 = CPose3D(itNod2->second);
				gl_edges->appendLine(
					mrpt::math::TPoint3D(p1.x(), p1.y(), p1.z()),
					mrpt::math::TPoint3D(p2.x(), p2.y(), p2.z()));
			}
		}
		ret->insert(gl_edges);

	}  // end draw edges

	return ret;

	MRPT_TRY_END
}
}  // namespace mrpt::opengl::graph_tools
