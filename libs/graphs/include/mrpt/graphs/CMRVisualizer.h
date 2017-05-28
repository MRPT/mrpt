#ifndef CMRVISUALIZER_H
#define CMRVISUALIZER_H

#include <iostream>

#include <mrpt/graphs/CVisualizer.h>
#include <mrpt/graphs/TMRSlamNodeAnnotations.h>
#include <utility>

namespace mrpt { namespace graphs { namespace detail {

// (Dummy) standard version
//////////////////////////////////////////////////////////

/**\brief Wrapper class that provides visualization of a network of poses that
 * have been registered by many graphSLAM agents/robots
 *
 * \note This is a dummy 4 template arguments version of the CMRVisualizer
 * class just to silence GCC warnings on conversions.
 * For implementation details users should see its specialized version that
 * runs specifically for TMRSlamNodeAnnotations as the 3rd template argument.
 *
 */
template<
	class CPOSE, // Type of edges
	class MAPS_IMPLEMENTATION, // Use std::map<> vs. std::vector<>
	class NODE_ANNOTATIONS=mrpt::graphs::detail::TMRSlamNodeAnnotations,
	class EDGE_ANNOTATIONS=mrpt::graphs::detail::edge_annotations_empty
	>
class CMRVisualizer:
	public CVisualizer<CPOSE, MAPS_IMPLEMENTATION, NODE_ANNOTATIONS, EDGE_ANNOTATIONS>
{
public:
	typedef CVisualizer<CPOSE, MAPS_IMPLEMENTATION, NODE_ANNOTATIONS, EDGE_ANNOTATIONS>
		parent;
	typedef mrpt::graphs::CNetworkOfPoses<
		CPOSE,
		MAPS_IMPLEMENTATION,
		NODE_ANNOTATIONS,
		EDGE_ANNOTATIONS> GRAPH_T;

	CMRVisualizer(const GRAPH_T& graph_in);

	~CMRVisualizer();
	void drawNodePoints(mrpt::opengl::CSetOfObjectsPtr& object,
			const mrpt::utils::TParametersDouble* viz_params=NULL) const;
	void drawEdges(mrpt::opengl::CSetOfObjectsPtr& object,
			const mrpt::utils::TParametersDouble* viz_params=NULL) const;

private:
};

// Specialized version
//////////////////////////////////////////////////////////

template<
	class CPOSE, // Type of edges
	class MAPS_IMPLEMENTATION, // Use std::map<> vs. std::vector<>
	class EDGE_ANNOTATIONS
	>
class CMRVisualizer<CPOSE, MAPS_IMPLEMENTATION, TMRSlamNodeAnnotations, EDGE_ANNOTATIONS>:
	public CVisualizer<CPOSE, MAPS_IMPLEMENTATION, TMRSlamNodeAnnotations, EDGE_ANNOTATIONS>
{
public:
	typedef CVisualizer<CPOSE, MAPS_IMPLEMENTATION, TMRSlamNodeAnnotations, EDGE_ANNOTATIONS>
		parent;
	typedef mrpt::graphs::CNetworkOfPoses<
		CPOSE,
		MAPS_IMPLEMENTATION,
		TMRSlamNodeAnnotations,
		EDGE_ANNOTATIONS> GRAPH_T;

	CMRVisualizer(const GRAPH_T& graph_in);

	~CMRVisualizer();
	void drawNodePoints(mrpt::opengl::CSetOfObjectsPtr& object,
			const mrpt::utils::TParametersDouble* viz_params=NULL) const;
	void drawEdges(mrpt::opengl::CSetOfObjectsPtr& object,
			const mrpt::utils::TParametersDouble* viz_params=NULL) const;

private:
};



} } } // end of namespaces

#include <mrpt/graphs/CMRVisualizer_impl.h>

#endif /* end of include guard: CMRVISUALIZER_H */
