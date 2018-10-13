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

#include "CRegistrationDeciderOrOptimizer.h"

namespace mrpt::graphslam::deciders
{
/**\brief Interface for implementing node registration classes.
 *
 * CNodeRegistrationDecider provides the basic methods that have to exist in
 * every node registration decider class. For an example of inheriting from
 * this class see CFixedIntervalsNRD.
 *
 * \note As a naming convention, all the implemented node registration deciders
 * are suffixed with the NRD acronym.
 *
 * \ingroup mrpt_graphslam_grp
 */
template <class GRAPH_T>
class CNodeRegistrationDecider
	: public virtual mrpt::graphslam::CRegistrationDeciderOrOptimizer<GRAPH_T>
{
   public:
	/**\brief Handy typedefs */
	/**\{*/
	/**\brief Parent of current class */
	using parent_t = mrpt::graphslam::CRegistrationDeciderOrOptimizer<GRAPH_T>;
	/**\brief type of graph constraints */
	using constraint_t = typename GRAPH_T::constraint_t;
	/**\brief type of underlying poses (2D/3D). */
	using pose_t = typename GRAPH_T::constraint_t::type_value;
	using global_pose_t = typename GRAPH_T::global_pose_t;
	using inf_mat_t = mrpt::math::CMatrixFixedNumeric<
		double, constraint_t::state_length, constraint_t::state_length>;
	/**\}*/

	/**\brief Default class constructor.*/
	CNodeRegistrationDecider();
	/**\brief Default class destructor.*/
	~CNodeRegistrationDecider() override = default;
	/**\brief Getter method for fetching the currently estimated robot position.
	 *
	 * In single-robot situations this is most likely going to be the last
	 * registered node position + an position/uncertainty increment from that
	 * position
	 */
	virtual global_pose_t getCurrentRobotPosEstimation() const;
	void getDescriptiveReport(std::string* report_str) const override;

   protected:
	/**\brief Reset the given PDF method and assign a fixed high-certainty
	 * Covariance/Information matrix
	 */
	void resetPDF(constraint_t* c);
	/**\brief Check whether a new node should be registered in the
	 * graph.
	 *
	 * This should be the key-method in any implementation of this
	 * interface. Should call registerNewNodeAtEnd method if the registration
	 * condition is satisfied.
	 *
	 * \return True upon successful node registration in the graph
	 */
	virtual bool checkRegistrationCondition();
	/**\brief Utility methods for adding new poses to the graph.
	 */
	/**\{*/
	/** Add a new constraint at the end of the graph.
	 * \param[in] constraint Constraint transformation from the latest
	 * registered to the new node.
	 *
	 * \return True upon successful node registration.
	 */
	bool registerNewNodeAtEnd(const typename GRAPH_T::constraint_t& constraint);
	/**\brief Same goal as the previous method - uses the m_since_prev_node_PDF
	 * as the constraint at the end.
	 */
	bool registerNewNodeAtEnd();
	/**\brief Get a global_pose_t and fill the NODE_ANNOTATIONS-related fields
	 *
	 * \note Users are encouraged to override this method in case they have
	 * defined a new TNodeAnnotations struct and want to use this metadata in
	 * the graph nodes.
	 */
	virtual void addNodeAnnotsToPose(global_pose_t* pose) const;
	/**\}*/

	/**\brief Store the last registered NodeID.
	 *
	 * We don't store its pose since it will most likely change due to calls to
	 * the
	 * graph-optimization procedure / dijkstra_node_estimation
	 */
	mrpt::graphs::TNodeID m_prev_registered_nodeID;
	/**\brief Tracking the PDF of the current position of the robot with
	 * regards to the <b previous registered node</b>.
	 */
	constraint_t m_since_prev_node_PDF;
	/**\brief Initial information matrix for paths
	 *
	 * Large values for this indicate that I am sure of the corresponding
	 * (initial) pose
	 */
	inf_mat_t m_init_inf_mat;
};
}  // namespace mrpt::graphslam::deciders
#include "CNodeRegistrationDecider_impl.h"
