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
		const mrpt::utils::TParametersDouble* viz_params/*=NULL*/) { }

template<class CPOSE, class MAPS_IMPLEMENTATION, class NODE_ANNOTATIONS, class EDGE_ANNOTATIONS>
void CMRVisualizer<CPOSE, MAPS_IMPLEMENTATION, NODE_ANNOTATIONS, EDGE_ANNOTATIONS>::
drawEdges(mrpt::opengl::CSetOfObjectsPtr& object,
		const mrpt::utils::TParametersDouble* viz_params/*=NULL*/) { }

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
		const mrpt::utils::TParametersDouble* viz_params/*=NULL*/) {

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

		const TMRSlamNodeAnnotations* node_annots = dynamic_cast<const TMRSlamNodeAnnotations*>(&curr_node);
		ASSERT_(node_annots);
		std::string curr_strid = node_annots->agent_ID_str;

		// have I already found this agent_ID_str?
		if (strid_to_cloud.find(curr_strid) != strid_to_cloud.end()) { // already registered CPointCloud

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
	for (std::map<std::string, CPointCloudPtr>::iterator
			it = strid_to_cloud.begin();
			it != strid_to_cloud.end();
			++it) {

		object->insert(it->second);
	}


}

template<class CPOSE, class MAPS_IMPLEMENTATION, class EDGE_ANNOTATIONS>
void CMRVisualizer<CPOSE, MAPS_IMPLEMENTATION, TMRSlamNodeAnnotations, EDGE_ANNOTATIONS>::drawEdges(
		mrpt::opengl::CSetOfObjectsPtr& object,
		const mrpt::utils::TParametersDouble* viz_params/*=NULL*/) {

	// TODO - implement this
	parent::drawEdges(object, viz_params);

}


} } } // end of namespaces

#endif /* end of include guard: CMRVISUALIZER_IMPL_H */
