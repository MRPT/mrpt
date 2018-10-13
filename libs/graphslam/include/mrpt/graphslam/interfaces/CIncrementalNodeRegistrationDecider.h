/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/config/CConfigFileBase.h>

#include <mrpt/graphslam/interfaces/CNodeRegistrationDecider.h>

namespace mrpt::graphslam::deciders
{
/**\brief Incremental Node registration decider. Decider adds new nodes in the
 * graph in an incremental fashion (adding nodes at the end of the graph)
 *
 * ### .ini Configuration Parameters
 *
 * \htmlinclude config_params_preamble.txt
 *
 * - \b class_verbosity
 *   + \a Section       : NodeRegistrationDeciderParameters
 *   + \a default value : 1 (mrpt::system::LVL_INFO)
 *   + \a Required      : FALSE
 *
 * - \b registration_max_distance
 *  + \a Section       : NodeRegistrationDeciderParameters
 *  + \a Default value : 0.5 // meters
 *  + \a Required      : FALSE
 *
 * - \b registration_max_angle
 *  + \a Section       : NodeRegistrationDeciderParameters
 *  + \a Default value : 10 // degrees
 *  + \a Required      : FALSE
 *
 * \ingroup mrpt_graphslam_grp
 *
 */
template <class GRAPH_T>
class CIncrementalNodeRegistrationDecider
	: public virtual mrpt::graphslam::deciders::CNodeRegistrationDecider<
		  GRAPH_T>
{
   public:
	using constraint_t = typename GRAPH_T::constraint_t;
	using pose_t = typename GRAPH_T::constraint_t::type_value;
	using global_pose_t = typename GRAPH_T::global_pose_t;
	using decider_t = CIncrementalNodeRegistrationDecider<GRAPH_T>;
	using parent_t =
		mrpt::graphslam::deciders::CNodeRegistrationDecider<GRAPH_T>;

	CIncrementalNodeRegistrationDecider() = default;
	virtual ~CIncrementalNodeRegistrationDecider() = default;

	/**\name Registration Conditions Specifiers
	 */
	/**\{ */
	/**\brief If estimated position surpasses the registration max values since
	 * the previous registered node, register a new node in the graph.
	 *
	 * \return True on successful registration.
	 */
	virtual bool checkRegistrationCondition();
	virtual bool checkRegistrationConditionPose(
		const mrpt::poses::CPose2D& p1, const mrpt::poses::CPose2D& p2) const;
	virtual bool checkRegistrationConditionPose(
		const mrpt::poses::CPose3D& p1, const mrpt::poses::CPose3D& p2) const;
	/**\} */

	virtual void loadParams(const std::string& source_fname);
	virtual void printParams() const;
	virtual void getDescriptiveReport(std::string* report_str) const;

   protected:
	/**\brief Parameters structure for managing the relevant to the decider
	 * variables in a compact manner
	 */
	struct TParams : public mrpt::config::CLoadableOptions
	{
	   public:
		TParams() = default;
		~TParams() = default;

		void loadFromConfigFile(
			const mrpt::config::CConfigFileBase& source,
			const std::string& section);
		void dumpToTextStream(std::ostream& out) const;
		/**\brief Return a string with the configuration parameters
		 */
		void getAsString(std::string* params_out) const;
		std::string getAsString() const;

		// max values for new node registration
		double registration_max_distance;
		double registration_max_angle;
	};

	TParams params;

   private:
};
}  // namespace mrpt::graphslam::deciders
#include "CIncrementalNodeRegistrationDecider_impl.h"
