/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/config/CLoadableOptions.h>

#include <string>

namespace mrpt::graphslam
{
/**\brief Holds the data of an information path.
 *
 * Path comprises of nodes (TNodeID type) and constraints between them.
 * Only consecutive nodes are connected by edges, thus \em path.
 *
 * \sa mrpt::deciders::CLoopCloserERD
 * \ingroup mrpt_graphslam_grp
 */
template <class GRAPH_T = typename mrpt::graphs::CNetworkOfPoses2DInf>
struct TUncertaintyPath : public mrpt::config::CLoadableOptions
{
	using constraint_t = typename GRAPH_T::constraint_t;
	using pose_t = typename constraint_t::type_value;
	using self_t = TUncertaintyPath<GRAPH_T>;

	TUncertaintyPath();
	TUncertaintyPath(const mrpt::graphs::TNodeID& starting_node);
	TUncertaintyPath(
		const mrpt::graphs::TNodeID& starting_node,
		const mrpt::graphs::TNodeID& ending_node, const constraint_t& edge);
	~TUncertaintyPath() override = default;
	void clear();
	bool isEmpty() const;
	/**\brief Assert that the current path is between the given nodeIDs.
	 *
	 * Call to this method practically checks if the give nodes match the source
	 * and destination nodeIDs.
	 *
	 * \note Assertions will be executed only in \b Debug builds
	 *
	 * \exception std::runtime_error in case the conditions don't hold
	 */
	void assertIsBetweenNodeIDs(
		const mrpt::graphs::TNodeID& from,
		const mrpt::graphs::TNodeID& to) const;

	// no need to load anything..
	void loadFromConfigFile(
		const mrpt::config::CConfigFileBase& source,
		const std::string& section) override;
	void dumpToTextStream(std::ostream& out) const override;
	std::string getAsString() const;
	void getAsString(std::string* str) const;

	/**\brief Return the source node of this path */
	const mrpt::graphs::TNodeID& getSource() const;
	/**\brief Return the Destination node of this path */
	const mrpt::graphs::TNodeID& getDestination() const;
	double getDeterminant();
	/**\brief Test if the current path has a lower uncertainty than the other
	 * path.
	 *
	 * \return True if the current path does have a lower uncertainty
	 */
	bool hasLowerUncertaintyThan(const self_t& other) const;
	/**\brief add a new link in the current path.
	 *
	 * Add the node that the path traverses and the information matrix of
	 * the extra link
	 */
	void addToPath(const mrpt::graphs::TNodeID& node, const constraint_t& edge);
	self_t& operator+=(const self_t& other);
	// results...
	bool operator==(const self_t& other) const;
	bool operator!=(const self_t& other) const;

	friend std::ostream& operator<<(std::ostream& o, const self_t& obj)
	{
		o << obj.getAsString() << endl;
		return o;
	}

	/**\brief Nodes that the Path comprises of.
	 *
	 * Nodes in the path are added to the end of the vector.
	 */
	std::vector<mrpt::graphs::TNodeID> nodes_traversed;
	/**\brief Current path position + corresponding covariance */
	constraint_t curr_pose_pdf;

	/**Determine whether the determinant of the Path is up-to-date and
	 * can be directly fetched or has to be computed again */
	/**\{*/
	bool determinant_is_updated;
	double determinant_cached;
	/**\}*/
};
}  // namespace mrpt::graphslam
#include "TUncertaintyPath_impl.h"
