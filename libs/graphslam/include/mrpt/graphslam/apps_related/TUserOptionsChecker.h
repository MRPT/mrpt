/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef TUSEROPTIONSCHECKER_H
#define TUSEROPTIONSCHECKER_H

#include <mrpt/system/filesystem.h>
#include <mrpt/utils/mrpt_macros.h>
#include <mrpt/graphs/CNetworkOfPoses.h>

#include <mrpt/graphslam/link_pragmas.h>
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

namespace mrpt { namespace graphslam { namespace detail {

/**\brief Properties struct for both the Registration Decider and Optimizer
	* classes
	*/
struct GRAPHSLAM_IMPEXP TRegistrationDeciderOrOptimizerProps {
	TRegistrationDeciderOrOptimizerProps():
		name(""),
		description(""),
		is_mr_slam_class(false) {}
	~TRegistrationDeciderOrOptimizerProps() {}

  /**\brief Name of the decider or optimizer class
   */
  std::string name;
  /**\brief General description of the decicder or optimizer class*/
  std::string description;
  /**\brief Class indicating if the current decider/optimizer class can be used
   * in a multi-robot SLAM operation
   */
  bool is_mr_slam_class;

};

/**\brief Properties struct for the Registration Decider classes.
 *
 * \ingroup mrpt_graphslam_grp
 */
struct GRAPHSLAM_IMPEXP TRegistrationDeciderProps :
	public TRegistrationDeciderOrOptimizerProps
{
	TRegistrationDeciderProps():
		type(""),
		rawlog_format("") {}
	~TRegistrationDeciderProps() {}

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
struct GRAPHSLAM_IMPEXP TOptimizerProps :
	public TRegistrationDeciderOrOptimizerProps
{
	TOptimizerProps() {}
	~TOptimizerProps() {}
};


/**\brief Class containing the declarations of supplementary methods that can
 * be used in application-related code.
 * Class instance can be handy for adding keeping the available
 * deciders/optimizers in a compact manner and for verifying whether a given
 * decider can be used.
 *
 * \ingroup mrpt_graphslam_grp
 */
struct GRAPHSLAM_IMPEXP TUserOptionsChecker {
	/**\name handy typedefs for the creation of deciders/optimzer instances from
	 * the corresponding strings
	 */
	/**\{*/
	typedef std::map<
		std::string,
		mrpt::graphslam::deciders::CNodeRegistrationDecider<
			mrpt::graphs::CNetworkOfPoses2DInf>*(*)()> node_regs_t;
	typedef std::map<
		std::string,
		mrpt::graphslam::deciders::CEdgeRegistrationDecider<
			mrpt::graphs::CNetworkOfPoses2DInf>*(*)()> edge_regs_t;
	typedef std::map<
		std::string,
		mrpt::graphslam::optimizers::CGraphSlamOptimizer<
			mrpt::graphs::CNetworkOfPoses2DInf>*(*)()> optimizers_t;
	/**\}*/

	//
	// methods
	//
	/**\brief Constructor */
	TUserOptionsChecker();
	/**\brief Destructor */
	virtual ~TUserOptionsChecker();
	/**\brief Create the necessary mappings from strings to the corresponding
	 * instance creation funtors.
	 *
	 * \note Method is by default called upon initialization
	 */
	virtual void createDeciderOptimizerMappings();
	/**\brief Populate the available deciders, optimizer classes available in
	 * user applications
	 *
	 * \note Method is by default called upon initialization
	 */
	virtual void populateDeciderOptimizerProperties();
	/**\brief Check if the given registrator decider exists in the vector of
 	 * deciders.
 	 * \param[in] given_reg String specifying the type of decider - This should
 	 * either be "node" or "edge"
 	 * \return True if it exists, false otherwise
 	 */
	virtual bool checkRegistrationDeciderExists(
			std::string given_reg,
			std::string reg_type) const;

	/**\brief Check if the given optimizer exists in the vector of optimizers.
 	 * \return True if it exists, false otherwise
 	 */
	virtual bool checkOptimizerExists(
			std::string given_opt) const;

	/**\brief Print the registration deciders vector in a formatted manner to the
 	 * standard output
 	 * \param[in] reg_type Method prints both the node registration
 	 * and edge registration deciders of the given vector unless specified
 	 * otherwise. The available argument options are "node", "edge", "all"
 	 */
	virtual void dumpRegistrarsToConsole(std::string reg_type="all") const;
	/**\brief Print the optimizers vector in a formatted manner to the standard
 	 * output.
 	 */
	virtual void dumpOptimizersToConsole() const;

	/**\name Methods for initializing decider/optimizer instances based on the user
 	 * command line choices - http://stackoverflow.com/a/582456/2843583
 	 *
 	 * \warning Caller is responsible for deleting the initialized instances
 	 */
	/**\{*/
	template<typename T>
	static mrpt::graphslam::deciders::CNodeRegistrationDecider<mrpt::graphs::CNetworkOfPoses2DInf>*
	createNodeRegistrationDecider() {
		return new T;
	}
	template<typename T>
	static mrpt::graphslam::deciders::CEdgeRegistrationDecider<mrpt::graphs::CNetworkOfPoses2DInf>*
	createEdgeRegistrationDecider() {
		return new T;
	}
	template<typename T>
	static mrpt::graphslam::optimizers::CGraphSlamOptimizer<mrpt::graphs::CNetworkOfPoses2DInf>*
	createGraphSlamOptimizer() {
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


} } } // END OF NAMESPACES

#endif /* end of include guard: TUSEROPTIONSCHECKER_H */
