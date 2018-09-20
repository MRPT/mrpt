/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/system/filesystem.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/poses/CPosePDFGaussianInf.h>
#include <mrpt/poses/CPose3DPDFGaussianInf.h>

#include <mrpt/graphslam/NRD/CFixedIntervalsNRD.h>
#include <mrpt/graphslam/NRD/CEmptyNRD.h>
#include <mrpt/graphslam/NRD/CICPCriteriaNRD.h>
#include <mrpt/graphslam/ERD/CICPCriteriaERD.h>
#include <mrpt/graphslam/ERD/CEmptyERD.h>
#include <mrpt/graphslam/ERD/CLoopCloserERD.h>
#include <mrpt/graphslam/GSO/CLevMarqGSO.h>

#include <string>
#include <iostream>
#include <vector>

namespace mrpt::graphslam::apps
{
/**\brief Properties struct for both the Registration Decider and Optimizer
 * classes
 */
struct TRegistrationDeciderOrOptimizerProps
{
	TRegistrationDeciderOrOptimizerProps() = default;
	~TRegistrationDeciderOrOptimizerProps() = default;
	/**\brief Name of the decider or optimizer class
	 */
	std::string name;
	/**\brief General description of the decicder or optimizer class*/
	std::string description;
	/**\brief Class indicating if the current decider/optimizer class can be
	 * used
	 * in a multi-robot SLAM operation
	 */
	bool is_mr_slam_class{false};
	bool is_slam_2d{true};
	bool is_slam_3d{false};
};

/**\brief Properties struct for the Registration Decider classes.
 *
 * \ingroup mrpt_graphslam_grp
 */
struct TRegistrationDeciderProps : public TRegistrationDeciderOrOptimizerProps
{
	TRegistrationDeciderProps() = default;
	~TRegistrationDeciderProps() = default;
	/**\brief Type of decider.
	 *
	 * Available options are:
	 * - node
	 * - edge
	 */
	std::string type;
	/**\brief Rawlog formats that the decider can be used in */
	std::string rawlog_format;
	/**\brief Measurements that the current decider class can utilize */
	std::vector<std::string> observations_used;
};

/**\brief Properties struct for the Optimizer classes.
 *
 * \ingroup mrpt_graphslam_grp
 */
struct TOptimizerProps : public TRegistrationDeciderOrOptimizerProps
{
	TOptimizerProps() = default;
	~TOptimizerProps() = default;
};

/**\brief Class containing the declarations of supplementary methods that can
 * be used in application-related code.
 * Class instance can be handy for adding keeping the available
 * deciders/optimizers in a compact manner and for verifying whether a given
 * decider can be used.
 *
 * \ingroup mrpt_graphslam_grp
 */
template <class GRAPH_t>
struct TUserOptionsChecker
{
	/**\name handy typedefs for the creation of deciders/optimzer instances from
	 * the corresponding strings
	 */
	/**\{*/
	using constraint_t = typename GRAPH_t::constraint_t;
	using pose_t = typename GRAPH_t::constraint_t::type_value;
	using node_regs_t = std::map<
		std::string,
		mrpt::graphslam::deciders::CNodeRegistrationDecider<GRAPH_t>* (*)()>;
	using edge_regs_t = std::map<
		std::string,
		mrpt::graphslam::deciders::CEdgeRegistrationDecider<GRAPH_t>* (*)()>;
	using optimizers_t = std::map<
		std::string,
		mrpt::graphslam::optimizers::CGraphSlamOptimizer<GRAPH_t>* (*)()>;

	/**\}*/

	//
	// methods
	//
	/**\brief Constructor */
	TUserOptionsChecker();
	/**\brief Destructor */
	virtual ~TUserOptionsChecker();
	/**\brief Create the necessary mappings from strings to the corresponding
	 * instance creation functors.
	 *
	 * Method is used for populating a map from string to instance creation
	 * function. The decider/optimzer can then be spawned according to the user
	 * selection.
	 *
	 */
	virtual void createDeciderOptimizerMappings();
	virtual void _createDeciderOptimizerMappings();
	/**\brief Populate the available decider, optimizer classes available in
	 * user applications
	 */
	virtual void populateDeciderOptimizerProperties();
	/**\brief Check if the given registrator decider exists in the vector of
	 * deciders.
	 * \param[in] given_reg String specifying the type of decider - This should
	 * either be "node" or "edge"
	 * \return True if it exists, false otherwise
	 */
	virtual bool checkRegistrationDeciderExists(
		std::string given_reg, std::string reg_type) const;

	/**\brief Check if the given optimizer exists in the vector of optimizers.
	 * \return True if it exists, false otherwise
	 */
	virtual bool checkOptimizerExists(std::string given_opt) const;

	/**\brief Print the registration deciders vector in a formatted manner to
	 * the
	 * standard output
	 * \param[in] reg_type Method prints both the node registration
	 * and edge registration deciders of the given vector unless specified
	 * otherwise. The available argument options are "node", "edge", "all"
	 */
	virtual void dumpRegistrarsToConsole(std::string reg_type = "all") const;
	/**\brief Print the optimizers vector in a formatted manner to the standard
	 * output.
	 */
	virtual void dumpOptimizersToConsole() const;

	/**\name Methods for initializing decider/optimizer instances based on the
	 * user
	 * command line choices - http://stackoverflow.com/a/582456/2843583
	 *
	 * \warning Caller is responsible for deleting the initialized instances
	 */
	/**\{*/
	template <class T>
	static mrpt::graphslam::deciders::CNodeRegistrationDecider<GRAPH_t>*
		createNodeRegistrationDecider()
	{
		return new T;
	}
	template <class T>
	static mrpt::graphslam::deciders::CEdgeRegistrationDecider<GRAPH_t>*
		createEdgeRegistrationDecider()
	{
		return new T;
	}
	template <class T>
	static mrpt::graphslam::optimizers::CGraphSlamOptimizer<GRAPH_t>*
		createGraphSlamOptimizer()
	{
		return new T;
	}
	/**}*/

	/**\name Mappings from registration decider/optimizer names to functors
	 * for creating instances of the corresponding classes
	 */
	/**\{*/
	node_regs_t node_regs_map;
	edge_regs_t edge_regs_map;
	optimizers_t optimizers_map;
	/**\}*/

	/**\name Vectors containing descriptions about the available
	 * deciders/optimizers.
	 *
	 * Handy for displaying information to the user (e.g. in help text)
	 */
	/**\{*/
	std::vector<TRegistrationDeciderProps*> regs_descriptions;
	std::vector<TOptimizerProps*> optimizers_descriptions;
	/**\}*/

	const std::string sep_header;
	const std::string sep_subheader;
};
}  // namespace mrpt::graphslam::apps
#include "TUserOptionsChecker_impl.h"
