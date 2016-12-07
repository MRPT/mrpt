/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT)               |
	 |                          http://www.mrpt.org/                             |
	 |                                                                           |
	 | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
	 | See: http://www.mrpt.org/Authors - All rights reserved.                   |
	 | Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+ */

#ifndef TUNCERTAINTYPATH_H
#define TUNCERTAINTYPATH_H


#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/math/CMatrix.h>
#include <mrpt/math/data_utils.h>
#include <mrpt/math/utils.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/types_simple.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>

#include <string>
#include <sstream>

namespace mrpt { namespace graphslam {

/**\brief Holds the data of an information path.
 *
 * Path comprises of nodes (TNodeID type) and constraints between them.
 * Only consecutive nodes are connected by edges, thus \em path.
 *
 * \sa mrpt::deciders::CLoopCloserERD
 * \ingroup mrpt_graphslam_grp
 */
template<class GRAPH_t=typename mrpt::graphs::CNetworkOfPoses2DInf>
struct TUncertaintyPath : public mrpt::utils::CLoadableOptions {
	/**\brief Handy typedefs */
	/**\{*/
	/**\brief type of graph constraints */
	typedef typename GRAPH_t::constraint_t constraint_t;
	/**\brief type of underlying poses (2D/3D). */
	typedef typename constraint_t::type_value pose_t;
	/**\}*/

	// methods
	// ////////////////////////////
	TUncertaintyPath();
	TUncertaintyPath(mrpt::utils::TNodeID starting_node);
	~TUncertaintyPath();
	void clear();

	// no need to load anything..
	void loadFromConfigFile(
			const mrpt::utils::CConfigFileBase &source,
			const std::string &section);
	void 	dumpToTextStream(mrpt::utils::CStream &out) const;
	std::string getAsString() const;
	void getAsString(std::string* str) const;

	/**\brief Return the source node of this path */
	mrpt::utils::TNodeID getSource() const;
	/**\brief Return the Destination node of this path */
	mrpt::utils::TNodeID getDestination() const;
	double getDeterminant();
	/**\brief Test if the current path has a lower uncertainty than the other
	 * path.
	 *
	 * \return True if the current path does have a lower uncertainty
	 */
	bool hasLowerUncertaintyThan(
			const TUncertaintyPath<GRAPH_t>& other) const;
	/**\brief add a new link in the current path.
	 *
	 * Add the node that the path traverses and the information matrix of
	 * the extra link
	 */
	void addToPath(mrpt::utils::TNodeID node, typename GRAPH_t::constraint_t edge);
	/**brief Test weather the constraints are of type CPosePDFGaussianInf.*/
	bool isGaussianInfType() const;
	/**brief Test weather the constraints are of type CPosePDFGaussian.  */
	bool isGaussianType() const;

	TUncertaintyPath<GRAPH_t>& operator+=(
			const TUncertaintyPath<GRAPH_t>& other);
	// results...
	bool operator==(const TUncertaintyPath<GRAPH_t>& other);
	bool operator!=(const TUncertaintyPath<GRAPH_t>& other);

	/**\brief Nodes that the Path comprises of.
	 *
	 * Nodes in the path are added to the end of the vector.
	 */
	std::vector<mrpt::utils::TNodeID> nodes_traversed;
	/**\brief Current path position + corresponding covariance */
	typename GRAPH_t::constraint_t curr_pose_pdf;

	/**Determine whether the determinant of the Path is up-to-date and
	 * can be directly fetched or has to be computed again */
	/**\{*/
	bool determinant_is_updated;
	double determinant_cached;
	/**\}*/

};

} } // end of namespaces

#include "TUncertaintyPath_impl.h"

#endif /* end of include guard: TUNCERTAINTYPATH_H */
