/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/graphs/CNetworkOfPoses.h>

#include "CRegistrationDeciderOrOptimizer.h"

#include <map>
#include <string>

namespace mrpt::graphslam::deciders
{
/** \brief Interface for implementing edge registration classes.
 *
 * CEdgeRegistrationDecider provides the basic methods that have to exist in
 * every edge registration decider class. For an example of inheriting from
 * this class see CICPCriteriaERD.
 *
 * \note As a naming convention, all the implemented edge registration deciders
 * are suffixed with the ERD acronym.
 *
 * \ingroup mrpt_graphslam_grp
 */
template <class GRAPH_T = typename mrpt::graphs::CNetworkOfPoses2DInf>
class CEdgeRegistrationDecider
	: public virtual mrpt::graphslam::CRegistrationDeciderOrOptimizer<GRAPH_T>
{
   public:
	/**\brief Handy typedefs */
	/**\{*/
	/**\brief Parent of current class */
	using parent = mrpt::graphslam::CRegistrationDeciderOrOptimizer<GRAPH_T>;
	/**\brief type of graph constraints */
	using constraint_t = typename GRAPH_T::constraint_t;
	/**\brief type of underlying poses (2D/3D). */
	using pose_t = typename GRAPH_T::constraint_t::type_value;
	/**\}*/

	CEdgeRegistrationDecider() = default;
	~CEdgeRegistrationDecider() override = default;
	/**\brief Fill the given map with the type of registered edges as well as
	 * the corresponding number of registration of each edge.
	 */
	virtual void getEdgesStats(
		std::map<std::string, int>* edge_type_to_num) const {};
	/**\brief Used by the caller to query for possible loop closures in the
	 * last edge registration procedure.
	 */
	virtual bool justInsertedLoopClosure() const { return m_just_inserted_lc; }
	void getDescriptiveReport(std::string* report_str) const override;

   protected:
	/**\name Registration criteria checks
	 *\brief Check whether a new edge should be registered in the
	 * graph.
	 *
	 * If condition(s) for edge registration is satisfied, method should
	 * call the registerNewEdge method.
	 */
	/**\{*/
	virtual void checkRegistrationCondition(
		mrpt::graphs::TNodeID from, mrpt::graphs::TNodeID to)
	{
	}
	virtual void checkRegistrationCondition(
		const std::set<mrpt::graphs::TNodeID>&)
	{
	}
	/**\}*/
	/**\brief Register a new constraint/edge in the current graph.
	 *
	 * Implementations of this class should provide a wrapper around
	 * GRAPH_T::insertEdge method.
	 */
	virtual void registerNewEdge(
		const mrpt::graphs::TNodeID& from, const mrpt::graphs::TNodeID& to,
		const constraint_t& rel_edge);

	bool m_just_inserted_lc = false;
	/**\brief Indicates whether the ERD implementation expects, at most one
	 * single node to be registered, between successive calls to the
	 * updateState method.
	 */
	bool m_override_registered_nodes_check = false;
};
}  // namespace mrpt::graphslam::deciders
#include "CEdgeRegistrationDecider_impl.h"
