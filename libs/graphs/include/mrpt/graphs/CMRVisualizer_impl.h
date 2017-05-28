#ifndef CMRVISUALIZER_IMPL_H
#define CMRVISUALIZER_IMPL_H

#include <mrpt/utils/TColorManager.h>

namespace mrpt { namespace graphs { namespace detail {


// (Dummy) standard version
// vvvvvvvvvvvvvvvvvvvvvvvv
//////////////////////////////////////////////////////////

template<class CPOSE, class MAPS_IMPLEMENTATION, class NODE_ANNOTATIONS, class EDGE_ANNOTATIONS>
CMRVisualizer<CPOSE, MAPS_IMPLEMENTATION, NODE_ANNOTATIONS, EDGE_ANNOTATIONS>::
CMRVisualizer(const GRAPH_T& graph_in):
	parent(graph_in)
{
	THROW_EXCEPTION(
			"CMRVisualizer standard (non-specialized) edition doesn't server any role."
			"In case you use this visualizer specify TMRSlamNodeAnnotations"
			"as the 3rd template argument");
}

template<class CPOSE, class MAPS_IMPLEMENTATION, class NODE_ANNOTATIONS, class EDGE_ANNOTATIONS>
CMRVisualizer<CPOSE, MAPS_IMPLEMENTATION, NODE_ANNOTATIONS, EDGE_ANNOTATIONS>::
~CMRVisualizer() { }

template<class CPOSE, class MAPS_IMPLEMENTATION, class NODE_ANNOTATIONS, class EDGE_ANNOTATIONS>
void CMRVisualizer<CPOSE, MAPS_IMPLEMENTATION, NODE_ANNOTATIONS, EDGE_ANNOTATIONS>::
drawNodePoints(mrpt::opengl::CSetOfObjectsPtr& object,
		const mrpt::utils::TParametersDouble* viz_params/*=NULL*/) const { }

template<class CPOSE, class MAPS_IMPLEMENTATION, class NODE_ANNOTATIONS, class EDGE_ANNOTATIONS>
void CMRVisualizer<CPOSE, MAPS_IMPLEMENTATION, NODE_ANNOTATIONS, EDGE_ANNOTATIONS>::
drawEdges(mrpt::opengl::CSetOfObjectsPtr& object,
		const mrpt::utils::TParametersDouble* viz_params/*=NULL*/) const { }

// ^^^^^^^^^^^^^^^^^^^^^^^^
// (Dummy Standard version ends here.
//////////////////////////////////////////////////////////


// Specialized version for the multi-robot case
//////////////////////////////////////////////////////////

template<class CPOSE, class MAPS_IMPLEMENTATION, class EDGE_ANNOTATIONS>
CMRVisualizer<CPOSE, MAPS_IMPLEMENTATION, TMRSlamNodeAnnotations, EDGE_ANNOTATIONS>::
CMRVisualizer(const GRAPH_T& graph_in):
	parent(graph_in)
{ }

template<class CPOSE, class MAPS_IMPLEMENTATION, class EDGE_ANNOTATIONS>
CMRVisualizer<CPOSE, MAPS_IMPLEMENTATION, TMRSlamNodeAnnotations, EDGE_ANNOTATIONS>::
~CMRVisualizer() { }


template<class CPOSE, class MAPS_IMPLEMENTATION, class EDGE_ANNOTATIONS>
void CMRVisualizer<CPOSE, MAPS_IMPLEMENTATION, TMRSlamNodeAnnotations, EDGE_ANNOTATIONS>::
drawNodePoints(mrpt::opengl::CSetOfObjectsPtr& object,
		const mrpt::utils::TParametersDouble* viz_params/*=NULL*/) const {

	using namespace mrpt::opengl;
	using namespace mrpt::utils;
	using namespace mrpt::graphs;
	using namespace std;

	const double nodes_point_size = viz_params->getWithDefaultVal(
			"nodes_point_size", 0.);

	// in case this is a combination of graphs like in multiple-robot
	// graphSLAM applications, use a unique color for all the nodes that
	// have initially been registered by each graphSLAM agent.

	TColorManager nodes_color_mngr;
	// map of agent string identifier to its corresponding point cloud
	map<string, CPointCloudPtr> strid_to_cloud;
	// map of agent string identifier to its corresponding CPointCloud color
	map<string, TColorf> strid_to_color;

	// traverse all nodes; Register a new pair to the aforementioned maps
	// if a node of a (still) unknown agent is traversed
	for (typename GRAPH_T::global_poses_t::const_iterator
			n_it = this->m_graph.nodes.begin();
			n_it != this->m_graph.nodes.end();
			++n_it) {

		const typename GRAPH_T::global_pose_t curr_node = n_it->second;

		//const TMRSlamNodeAnnotations* node_annots = dynamic_cast<const TMRSlamNodeAnnotations*>(&curr_node);
		//ASSERT_(node_annots);
		//std::string curr_strid = node_annots->agent_ID_str;
		const string& curr_strid = curr_node.agent_ID_str;

		// have I already found this agent_ID_str?
		if (strid_to_cloud.find(curr_strid) != strid_to_cloud.end()) {

			// if the CPointCloud is registered, its color must also be registered.
			ASSERTMSG_(strid_to_color.find(curr_strid) != strid_to_color.end(),
					"Agent string ID not found in colors map even though its CPointCloud exists.");

		}
		else { // CPointCloud not yet registered.
			// Create CPointCloud
			strid_to_cloud.insert(make_pair(
						curr_strid, CPointCloud::Create()));
			// Create TColorf
			strid_to_color.insert(make_pair(
						curr_strid, nodes_color_mngr.getNextTColorf()));

			CPointCloudPtr& curr_cloud  = strid_to_cloud.at(curr_strid);
			curr_cloud->setColor(strid_to_color.at(curr_strid));
			curr_cloud->setPointSize(nodes_point_size);
			curr_cloud->enablePointSmooth();

		} // end of (is CPointCloud/Color registered)

		// CPointCloud is initialized
		const CPose3D p = CPose3D(n_it->second); // Convert to 3D from whatever its real type.

		// insert current pose to its corresponding CPointCloud instance
		CPointCloudPtr& curr_cloud  = strid_to_cloud.at(curr_strid);
		curr_cloud->insertPoint(p.x(),p.y(), p.z());

	} // end for - nodes loop

	// insert all CPointCloud(s)
	for (std::map<std::string, CPointCloudPtr>::const_iterator
			it = strid_to_cloud.begin();
			it != strid_to_cloud.end();
			++it) {

		object->insert(it->second);
	}

} // end of drawNodePoints

template<class CPOSE, class MAPS_IMPLEMENTATION, class EDGE_ANNOTATIONS>
void CMRVisualizer<CPOSE, MAPS_IMPLEMENTATION, TMRSlamNodeAnnotations, EDGE_ANNOTATIONS>::
drawEdges(mrpt::opengl::CSetOfObjectsPtr& object,
		const mrpt::utils::TParametersDouble* viz_params/*=NULL*/) const {

	using namespace mrpt::opengl;
	using namespace mrpt::utils;

	ASSERTMSG_(viz_params, "Pointer to viz_params was not provided.");
	typedef map<pair<string, bool>, CSetOfLinesPtr> PairToSetOfLines_t;

	//Color manager instance managing the used colors for the registered edges
	TColorManager edges_color_mngr;
	// map of <agent string identifier, is_interconnecting_edge>
	// to its corresponding CSetOfLines pointer instance
	PairToSetOfLines_t id_to_set_of_lines;
	// map of agent string identifier to its corresponding CPointCloud color
	map<string, TColorf> strid_to_color;

	const double edge_width = viz_params->getWithDefaultVal(
			"edge_width", 2.);
	// width of the edges connecting different the nodes registered by two different agents
	// They are of outmost importance
	const double interconnecting_edge_width = viz_params->getWithDefaultVal(
			"interconnecting_edge_width", 4.);

	// traverse all edges
	for (typename GRAPH_T::const_iterator
			edge_it = this->m_graph.begin();
			edge_it != this->m_graph.end();
			++edge_it) {

		const TNodeID& start_node = edge_it->first.first;
		const TNodeID& end_node = edge_it->first.second;

		// iterator objects to the start and end node instances
		typename GRAPH_T::global_poses_t::const_iterator n_it1 =
			this->m_graph.nodes.find(start_node);
		typename GRAPH_T::global_poses_t::const_iterator n_it2 =
			this->m_graph.nodes.find(end_node);

		// Draw only if we have the global coords of both start and end nodes:
		if (n_it1 == this->m_graph.nodes.end() || n_it2 == this->m_graph.nodes.end()) {
			// skipping current edge...
			continue;
		}

		// the CSetOfLines of the current edge depends only on the combination of:
		// - agent_ID_str field of the **end node** 
		// - Whether the edge is an interconnection between two nodes having
		// different agent_ID_str values.
		std::string curr_end_strid = n_it2->second.agent_ID_str;
		bool is_interconnecting_edge = n_it1->second.agent_ID_str != n_it2->second.agent_ID_str;

		pair<string, bool> curr_pair = make_pair(curr_end_strid, is_interconnecting_edge);

		// have I already found the current pair
		if (id_to_set_of_lines.find(curr_pair) != id_to_set_of_lines.end()) {
		}
		else { // CSetOfLines not yet registered.
			// Register a new CSetOfLines when a unique pair of
			// <agent_ID_str (of end_node), is_interconnecting_edge> is found
			id_to_set_of_lines.insert(make_pair(
						curr_pair, CSetOfLines::Create()));

			// Create TColorf if not in map
			// Color depends only on the agent_ID_str
			if (strid_to_color.find(curr_end_strid) == strid_to_color.end()) {
				strid_to_color.insert(make_pair(
							curr_end_strid, edges_color_mngr.getNextTColorf()));
			}

			// both the CSetOfLines and TColorf entries should exist in their
			// corresponding maps by now
			CSetOfLinesPtr& curr_set_of_lines =
				id_to_set_of_lines.at(curr_pair);

			// color of the line
			curr_set_of_lines->setColor(strid_to_color.at(curr_end_strid));

			// width of the line
			double curr_width =
					is_interconnecting_edge? interconnecting_edge_width : edge_width;
			curr_set_of_lines->setLineWidth(curr_width);

		} // end of (is CSetOfLines/Color registered)

		// CSetOfLines is initialized

		// insert current edge to its corresponding CSetOfLines instance
		CSetOfLinesPtr& curr_set_of_lines = id_to_set_of_lines.at(curr_pair);
		const CPose3D p1 = CPose3D(n_it1->second);
		const CPose3D p2 = CPose3D(n_it2->second);
		curr_set_of_lines->appendLine(
					mrpt::math::TPoint3D(p1.x(),p1.y(),p1.z()),
					mrpt::math::TPoint3D(p2.x(),p2.y(),p2.z()));

	} // end for - nodes loop

	// insert all CSetOfLines(s)
	for (PairToSetOfLines_t::const_iterator
			it = id_to_set_of_lines.begin();
			it != id_to_set_of_lines.end();
			++it) {

		object->insert(it->second);
	}
} // end of drawEdges


} } } // end of namespaces

#endif /* end of include guard: CMRVISUALIZER_IMPL_H */
