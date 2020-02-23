/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

namespace mrpt::graphs::detail
{
// constructor, destructor
////////////////////////////////////////////////////////////
template <
	class CPOSE, class MAPS_IMPLEMENTATION, class NODE_ANNOTATIONS,
	class EDGE_ANNOTATIONS>
CVisualizer<CPOSE, MAPS_IMPLEMENTATION, NODE_ANNOTATIONS, EDGE_ANNOTATIONS>::
	CVisualizer(const GRAPH_T& graph_in)
	: m_graph(graph_in)
{
	// Is a 2D or 3D graph network?
	using constraint_t = typename GRAPH_T::constraint_t;
	m_is_3D_graph = constraint_t::is_3D();
}
template <
	class CPOSE, class MAPS_IMPLEMENTATION, class NODE_ANNOTATIONS,
	class EDGE_ANNOTATIONS>
CVisualizer<CPOSE, MAPS_IMPLEMENTATION, NODE_ANNOTATIONS, EDGE_ANNOTATIONS>::
	~CVisualizer() = default;

// methods implementations
////////////////////////////////////////////////////////////
template <
	class CPOSE, class MAPS_IMPLEMENTATION, class NODE_ANNOTATIONS,
	class EDGE_ANNOTATIONS>
void CVisualizer<
	CPOSE, MAPS_IMPLEMENTATION, NODE_ANNOTATIONS, EDGE_ANNOTATIONS>::
	getAs3DObject(
		mrpt::opengl::CSetOfObjects::Ptr& object,
		mrpt::system::TParametersDouble viz_params) const
{
	using namespace mrpt::opengl;

	// graph visualization parameters
	const bool show_ID_labels =
		0 != viz_params.getWithDefaultVal("show_ID_labels", 0);
	const bool show_ground_grid =
		0 != viz_params.getWithDefaultVal("show_ground_grid", 1);
	const bool show_edges = 0 != viz_params.getWithDefaultVal("show_edges", 1);
	const bool show_node_corners =
		0 != viz_params.getWithDefaultVal("show_node_corners", 1);
	const bool show_edge_rel_poses =
		0 != viz_params.getWithDefaultVal("show_edge_rel_poses", 0);
	const double nodes_point_size =
		viz_params.getWithDefaultVal("nodes_point_size", 0.);

	if (show_ground_grid)
	{
		this->drawGroundGrid(object, &viz_params);
	}

	if (nodes_point_size > 0)
	{
		this->drawNodePoints(object, &viz_params);
	}

	if (show_node_corners || show_ID_labels)
	{
		this->drawNodeCorners(object, &viz_params);
	}

	if (show_edge_rel_poses)
	{
		this->drawEdgeRelPoses(object, &viz_params);
	}

	if (show_edges)
	{
		this->drawEdges(object, &viz_params);
	}
}

template <
	class CPOSE, class MAPS_IMPLEMENTATION, class NODE_ANNOTATIONS,
	class EDGE_ANNOTATIONS>
void CVisualizer<
	CPOSE, MAPS_IMPLEMENTATION, NODE_ANNOTATIONS, EDGE_ANNOTATIONS>::
	drawGroundGrid(
		mrpt::opengl::CSetOfObjects::Ptr& object,
		const mrpt::system::TParametersDouble* viz_params /*=NULL*/) const
{
	using namespace mrpt::opengl;
	ASSERTMSG_(viz_params, "Pointer to viz_params was not provided.");

	// Estimate bounding box.
	mrpt::math::TPoint3D BB_min(-10., -10., 0.), BB_max(10., 10., 0.);

	for (auto n_it = m_graph.nodes.begin(); n_it != m_graph.nodes.end(); ++n_it)
	{
		const CPose3D p = CPose3D(
			n_it->second);  // Convert to 3D from whatever its real type.

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
	object->insert(grid);
}

template <
	class CPOSE, class MAPS_IMPLEMENTATION, class NODE_ANNOTATIONS,
	class EDGE_ANNOTATIONS>
void CVisualizer<
	CPOSE, MAPS_IMPLEMENTATION, NODE_ANNOTATIONS, EDGE_ANNOTATIONS>::
	drawNodePoints(
		mrpt::opengl::CSetOfObjects::Ptr& object,
		const mrpt::system::TParametersDouble* viz_params /*=NULL*/) const
{
	ASSERTMSG_(viz_params, "Pointer to viz_params was not provided.");

	using namespace mrpt::opengl;
	using namespace mrpt::img;

	const double nodes_point_size =
		viz_params->getWithDefaultVal("nodes_point_size", 0.);
	const unsigned int nodes_point_color = viz_params->getWithDefaultVal(
		"nodes_point_color", (unsigned int)0xA0A0A0);

	CPointCloud::Ptr pnts = std::make_shared<CPointCloud>();
	pnts->setColor(TColorf(TColor(nodes_point_color)));
	pnts->setPointSize(nodes_point_size);

	// Add all nodesnodes:
	for (auto n_it = m_graph.nodes.begin(); n_it != m_graph.nodes.end(); ++n_it)
	{
		const CPose3D p = CPose3D(
			n_it->second);  // Convert to 3D from whatever its real type.
		pnts->insertPoint(p.x(), p.y(), p.z());
	}

	pnts->enablePointSmooth();
	object->insert(pnts);
}

template <
	class CPOSE, class MAPS_IMPLEMENTATION, class NODE_ANNOTATIONS,
	class EDGE_ANNOTATIONS>
void CVisualizer<
	CPOSE, MAPS_IMPLEMENTATION, NODE_ANNOTATIONS, EDGE_ANNOTATIONS>::
	drawNodeCorners(
		mrpt::opengl::CSetOfObjects::Ptr& object,
		const mrpt::system::TParametersDouble* viz_params /*=NULL*/) const
{
	using namespace mrpt::opengl;
	using mrpt::poses::CPose3D;

	ASSERTMSG_(viz_params, "Pointer to viz_params was not provided.");

	const bool show_node_corners =
		0 != viz_params->getWithDefaultVal("show_node_corners", 1);
	const bool show_ID_labels =
		0 != viz_params->getWithDefaultVal("show_ID_labels", 0);
	const double nodes_corner_scale =
		viz_params->getWithDefaultVal("nodes_corner_scale", 0.7);

	for (auto n_it = m_graph.nodes.begin(); n_it != m_graph.nodes.end(); ++n_it)
	{
		// Convert to 3D from whatever its real type.  CSetOfObjects::Ptr
		// gl_corner = show_node_corners ?
		const CPose3D p = CPose3D(n_it->second);
		CSetOfObjects::Ptr gl_corner =
			show_node_corners
				? (m_is_3D_graph ? stock_objects::CornerXYZSimple(
									   nodes_corner_scale, 1.0 /*line width*/)
								 : stock_objects::CornerXYSimple(
									   nodes_corner_scale, 1.0 /*line width*/))
				: std::make_shared<CSetOfObjects>();
		gl_corner->setPose(p);
		if (show_ID_labels)
		{  // don't show IDs twice!
			gl_corner->setName(
				format("%u", static_cast<unsigned int>(n_it->first)));
			gl_corner->enableShowName();
		}
		object->insert(gl_corner);
	}
}

template <
	class CPOSE, class MAPS_IMPLEMENTATION, class NODE_ANNOTATIONS,
	class EDGE_ANNOTATIONS>
void CVisualizer<
	CPOSE, MAPS_IMPLEMENTATION, NODE_ANNOTATIONS, EDGE_ANNOTATIONS>::
	drawEdgeRelPoses(
		mrpt::opengl::CSetOfObjects::Ptr& object,
		const mrpt::system::TParametersDouble* viz_params /*=NULL*/) const
{
	using namespace mrpt::opengl;
	using namespace mrpt::img;
	using mrpt::graphs::TNodeID;
	ASSERTMSG_(viz_params, "Pointer to viz_params was not provided.");

	const double nodes_edges_corner_scale =
		viz_params->getWithDefaultVal("nodes_edges_corner_scale", 0.4);
	const unsigned int edge_rel_poses_color = viz_params->getWithDefaultVal(
		"edge_rel_poses_color", (unsigned int)0x40FF8000);
	const TColor col8bit(
		edge_rel_poses_color & 0xffffff, edge_rel_poses_color >> 24);
	const double edge_width = viz_params->getWithDefaultVal("edge_width", 2.);

	for (auto edge_it = m_graph.begin(); edge_it != m_graph.end(); ++edge_it)
	{
		// Node ID of the source pose:
		const TNodeID node_id_start = m_graph.edges_store_inverse_poses
										  ? edge_it->first.second
										  : edge_it->first.first;

		// Draw only if we have the global coords of starting nodes:
		auto n_it = m_graph.nodes.find(node_id_start);
		if (n_it != m_graph.nodes.end())
		{
			const CPose3D pSource = CPose3D(n_it->second);
			// Create a set of objects at that pose and do the rest in relative
			// coords:
			mrpt::opengl::CSetOfObjects::Ptr gl_rel_edge =
				mrpt::opengl::CSetOfObjects::Create();
			gl_rel_edge->setPose(pSource);

			const typename GRAPH_T::constraint_no_pdf_t& edge_pose =
				edge_it->second.getPoseMean();
			const mrpt::poses::CPoint3D edge_pose_pt =
				mrpt::poses::CPoint3D(edge_pose);

			mrpt::opengl::CSetOfObjects::Ptr gl_edge_corner =
				(m_is_3D_graph
					 ? stock_objects::CornerXYZSimple(
						   nodes_edges_corner_scale, 1.0 /*line width*/)
					 : stock_objects::CornerXYSimple(
						   nodes_edges_corner_scale, 1.0 /*line width*/));

			gl_edge_corner->setPose(edge_pose);
			gl_rel_edge->insert(gl_edge_corner);

			mrpt::opengl::CSimpleLine::Ptr gl_line =
				mrpt::opengl::CSimpleLine::Create(
					0, 0, 0, edge_pose_pt.x(), edge_pose_pt.y(),
					edge_pose_pt.z());

			gl_line->setColor_u8(col8bit);
			gl_line->setLineWidth(edge_width);
			gl_rel_edge->insert(gl_line);

			object->insert(gl_rel_edge);
		}
	}
}

template <
	class CPOSE, class MAPS_IMPLEMENTATION, class NODE_ANNOTATIONS,
	class EDGE_ANNOTATIONS>
void CVisualizer<
	CPOSE, MAPS_IMPLEMENTATION, NODE_ANNOTATIONS, EDGE_ANNOTATIONS>::
	drawEdges(
		mrpt::opengl::CSetOfObjects::Ptr& object,
		const mrpt::system::TParametersDouble* viz_params /*=NULL*/) const
{
	using namespace mrpt::opengl;
	using namespace mrpt::img;
	using namespace mrpt::poses;
	ASSERTMSG_(viz_params, "Pointer to viz_params was not provided.");

	CSetOfLines::Ptr gl_edges = std::make_shared<CSetOfLines>();
	const unsigned int edge_color =
		viz_params->getWithDefaultVal("edge_color", (unsigned int)0x400000FF);
	const double edge_width = viz_params->getWithDefaultVal("edge_width", 2.);

	const TColor col8bit(edge_color & 0xffffff, edge_color >> 24);

	gl_edges->setColor_u8(col8bit);
	gl_edges->setLineWidth(edge_width);

	// for all registered edges.
	for (auto edge_it = m_graph.begin(); edge_it != m_graph.end(); ++edge_it)
	{
		const TNodeID id1 = edge_it->first.first;
		const TNodeID id2 = edge_it->first.second;

		// Draw only if we have the global coords of both nodes:
		auto n_it1 = m_graph.nodes.find(id1);
		auto n_it2 = m_graph.nodes.find(id2);
		if (n_it1 != m_graph.nodes.end() && n_it2 != m_graph.nodes.end())
		{  // both nodes found?
			const CPose3D p1 = CPose3D(n_it1->second);
			const CPose3D p2 = CPose3D(n_it2->second);
			gl_edges->appendLine(
				mrpt::math::TPoint3D(p1.x(), p1.y(), p1.z()),
				mrpt::math::TPoint3D(p2.x(), p2.y(), p2.z()));
		}
	}
	object->insert(gl_edges);
}
}  // namespace mrpt::graphs::detail
