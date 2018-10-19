/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/config/CConfigFileBase.h>

#include <mrpt/graphslam/interfaces/CNodeRegistrationDecider.h>

namespace mrpt::graphslam::deciders
{
/**\brief Fixed Intervals Odometry-based Node Registration
 *
 * ## Description
 *
 * Determine whether to insert a new pose in the graph given the distance and
 * angle thresholds. When the odometry readings indicate that any of the
 * thresholds has been surpassed, with regards to the previous registered
 * pose, a new node is added in the graph.
 *
 * Current decider is a minimal, simple implementation of the
 * CNodeRegistrationDecider interface which can be used for 2D datasets.
 * Decider *does not guarantee* thread safety when accessing the GRAPH_T
 * resource. This is handled by the CGraphSlamEngine instance.
 *
 * ### Specifications
 *
 * - Map type: 2D
 * - MRPT rawlog format: #1, #2
 * - Graph Type: CPosePDFGaussianInf
 * - Observations Used: CObservationOdometry, CActionRobotMovement2D
 * - Node Registration Strategy: Fixed Odometry Intervals
 *
 * ### .ini Configuration Parameters
 *
 * \htmlinclude graphslam-engine_config_params_preamble.txt
 *
 * - \b class_verbosity
 *   + \a Section       : NodeRegistrationDeciderParameters
 *   + \a Default value : 1 (mrpt::system::LVL_INFO)
 *   + \a Required      : FALSE
 *
 * - \b registration_max_distance
 *  + \a Section       : NodeRegistrationDeciderParameters
 *  + \a Default value : 0.5 // meters
 *  + \a Required      : FALSE
 *
 * - \b registration_max_angle
 *  + \a Section       : NodeRegistrationDeciderParameters
 *  + \a Default value : 60 // degrees
 *  + \a Required      : FALSE
 *
 * \ingroup mrpt_graphslam_grp
 */
template <class GRAPH_T = typename mrpt::graphs::CNetworkOfPoses2DInf>
class CFixedIntervalsNRD
	: public virtual mrpt::graphslam::deciders::CNodeRegistrationDecider<
		  GRAPH_T>
{
   public:
	// Public functions
	//////////////////////////////////////////////////////////////

	/**\brief Handy typedefs */
	/**\{*/
	/**\brief Node Registration Decider */
	using node_reg =
		mrpt::graphslam::deciders::CNodeRegistrationDecider<GRAPH_T>;

	/**\brief type of graph constraints */
	using constraint_t = typename GRAPH_T::constraint_t;
	/**\brief type of underlying poses (2D/3D). */
	using pose_t = typename GRAPH_T::constraint_t::type_value;
	using global_pose_t = typename GRAPH_T::global_pose_t;

	using inf_mat_t = mrpt::math::CMatrixFixedNumeric<
		double, constraint_t::state_length, constraint_t::state_length>;
	/**\brief Node Registration Decider */
	using parent_t =
		mrpt::graphslam::deciders::CNodeRegistrationDecider<GRAPH_T>;
	/**\}*/

	CFixedIntervalsNRD();
	~CFixedIntervalsNRD() override = default;

	void loadParams(const std::string& source_fname) override;
	void printParams() const override;
	void getDescriptiveReport(std::string* report_str) const override;

	/**\brief Method makes use of the CActionCollection/CObservation to update
	 * the
	 * odometry estimation from the last inserted pose
	 *
	 * \return True upon successful node registration in the graph
	 */
	bool updateState(
		mrpt::obs::CActionCollection::Ptr action,
		mrpt::obs::CSensoryFrame::Ptr observations,
		mrpt::obs::CObservation::Ptr observation) override;

	/**\brief Parameters structure for managing the relevant to the decider
	 * variables in a compact manner
	 */
	struct TParams : public mrpt::config::CLoadableOptions
	{
	   public:
		TParams() = default;
		~TParams() override = default;

		void loadFromConfigFile(
			const mrpt::config::CConfigFileBase& source,
			const std::string& section) override;
		void dumpToTextStream(std::ostream& out) const override;
		/**\brief Return a string with the configuration parameters
		 */
		void getAsString(std::string* params_out) const;
		std::string getAsString() const;

		// max values for new node registration
		double registration_max_distance;
		double registration_max_angle;
	};

	TParams params;

   protected:
	/**\name Registration Conditions Specifiers
	 */
	/**\brief If estimated position surpasses the registration max values since
	 * the previous registered node, register a new node in the graph.
	 *
	 * \return True on successful registration.
	 */
	/**\{ */
	bool checkRegistrationCondition() override;
	bool checkRegistrationCondition(
		const mrpt::poses::CPose2D& p1, const mrpt::poses::CPose2D& p2) const;
	bool checkRegistrationCondition(
		const mrpt::poses::CPose3D& p1, const mrpt::poses::CPose3D& p2) const;
	/**\} */

	/**\brief pose_t estimation using only odometry information. Handy for
	 * observation-only rawlogs.
	 */
	pose_t m_curr_odometry_only_pose;
	/**\brief pose_t estimation using only odometry information. Handy for
	 * observation-only rawlogs.
	 */
	pose_t m_last_odometry_only_pose;
	/**\brief Keep track of whether we are reading from an observation-only
	 * rawlog file or from an action-observation rawlog
	 */
	bool m_observation_only_rawlog;
};
}  // namespace mrpt::graphslam::deciders
#include "CFixedIntervalsNRD_impl.h"
