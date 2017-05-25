#ifndef CVISUALIZER_H
#define CVISUALIZER_H

#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CSimpleLine.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/utils/TParameters.h>
#include <mrpt/graphs/CNetworkOfPoses.h>

namespace mrpt { namespace graphs { namespace detail {

/**\brief Base class for C*Visualizer classes.
 *
 * By default provides visualization for a CNetowrkOfPoses containing nodes and
 * edges as constructed by a single agent/robot. Derived classes can inherit
 * and partially modify the class behavior 
*/
template<
	class CPOSE, // Type of edges
	class MAPS_IMPLEMENTATION, // Use std::map<> vs. std::vector<>
	class NODE_ANNOTATIONS=mrpt::graphs::detail::TNodeAnnotations,
	class EDGE_ANNOTATIONS=mrpt::graphs::detail::edge_annotations_empty
	>
class CVisualizer
{
public:
	typedef mrpt::graphs::CNetworkOfPoses<
		CPOSE, 
		MAPS_IMPLEMENTATION,
		NODE_ANNOTATIONS,
		EDGE_ANNOTATIONS> GRAPH_T; 
	/**\brief Constructor
	*/
	CVisualizer(const GRAPH_T& graph_in);
	/**\brief Destructor
	*/
	virtual ~CVisualizer();
	/**\brief Common visualization stuff for all derived classes
	 *
	 * Function delegates visualization tasks to the draw* methods according to
	 * the user preferences
	 *
	 */
	virtual void 	getAs3DObject(
			mrpt::opengl::CSetOfObjectsPtr& object,
			mrpt::utils::TParametersDouble viz_params) const;

protected:
	/**\name Work-splitting methods
	 * \brief Smaller functions that do add very specific parts to the visual representation
	 *
	 * Following functions take an optional TParametersDouble instance containing
	 * visualization parameters
	 *
	 */
	/**\{ */
	virtual void drawGroundGrid(mrpt::opengl::CSetOfObjectsPtr& object,
			const mrpt::utils::TParametersDouble* viz_params=NULL) const;
	virtual void drawNodeCorners(mrpt::opengl::CSetOfObjectsPtr& object,
			const mrpt::utils::TParametersDouble* viz_params=NULL) const;
	virtual void drawNodePoints(mrpt::opengl::CSetOfObjectsPtr& object,
			const mrpt::utils::TParametersDouble* viz_params=NULL) const;
	virtual void drawEdgeRelPoses(mrpt::opengl::CSetOfObjectsPtr& object,
			const mrpt::utils::TParametersDouble* viz_params=NULL) const;
	virtual void drawEdges(mrpt::opengl::CSetOfObjectsPtr& object,
			const mrpt::utils::TParametersDouble* viz_params=NULL) const;

	/**\} */

	bool m_is_3D_graph;
	const GRAPH_T& m_graph;
};


} } } // end of namespaces

#include <mrpt/graphs/CVisualizer_impl.h>

#endif /* end of include guard: CVISUALIZER_H */
