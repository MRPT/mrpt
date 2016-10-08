/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/graphslam/CNodeRegistrationDecider.h>
#include <mrpt/graphslam/CEdgeRegistrationDecider.h>
#include <mrpt/graphslam/CGraphSlamOptimizer.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/utils/mrpt_macros.h>

#include <iostream>
#include <string>
#include <vector>

/**\name File containing the declarations of supplementary functions that can be used
 * in application-related code
 *
 * \ingroup mrpt_graphslam_grp
 */
/**\{*/

/**\brief Namespace containing supplementary functions that can be used in
 * application-related code that makes use of the mrpt-graphslam API
 */
namespace mrpt { namespace graphslam { namespace supplementary {

/**\brief Properties struct for the Registration Decider classes. */
struct TRegistrationDeciderProps {
	TRegistrationDeciderProps() {}
	~TRegistrationDeciderProps() {}

  std::string name;
  std::string description;
  std::string type; // type of registration decider - node/edge?
  std::string rawlog_format; // rawlog formats that the decider can be used in
  std::vector<std::string> observations_used;
};

/**\brief Properties struct for the Optimizer classes. */
struct TOptimizerProps {
	TOptimizerProps() {}
	~TOptimizerProps() {}

	std::string name;
	std::string description;

};

/**\brief Check if the given registrator decider exists in the vector of
 * deciders.
 * \param[in] registrars_vec Vector with pointers to the avaiable registration
 * deciders
 * \param[in] given_reg String specifying the type of decider - This should
 * either be "node" or "edge"
 * \return True if it exists, false otherwise
 */
bool checkRegistrationDeciderExists(
		const std::vector<TRegistrationDeciderProps*>& registrars_vec,
		std::string given_reg,
		std::string reg_type);

/**\brief Check if the given optimizer exists in the vector of optimizers.
 * \param[in] optimizers_vec Vector with pointers to the avaiable optimizers
 * \return True if it exists, false otherwise
 */
bool checkOptimizerExists(
		const std::vector<TOptimizerProps*>& optimizers_vec,
		std::string given_opt);

/**\brief Print the registration deciders vector in a formatted manner to the
 * standard output
 * \param[in] registrars_vec Vector with the pointers to the available
 * registration deciders
 * \param[in] reg_type Function prints both the node registration
 * and edge registration deciders of the given vector unless specified
 * otherwise. The available argument options are "node", "edge", "all"
 */
void dumpRegistrarsToConsole(
		const std::vector<TRegistrationDeciderProps*>& registrars_vec,
		std::string reg_type="all"
		);

/**\brief Print the optimizers vector in a formatted manner to the standard
 * output.
 * \param[in] optimizers_vec Vector with the pointers to the available
 * optimizers
 */
void dumpOptimizersToConsole(
		const std::vector<TOptimizerProps*>& optimizers_vec);

/**\name Functions for initializing decider/optimizer instances based on the user
 * command line choices - http://stackoverflow.com/a/582456/2843583
 */
/**\{*/
template<typename T>
mrpt::graphslam::deciders::CNodeRegistrationDecider<mrpt::graphs::CNetworkOfPoses2DInf>*
createNodeRegistrationDecider() {
	return new T;
}
template<typename T>
mrpt::graphslam::deciders::CEdgeRegistrationDecider<mrpt::graphs::CNetworkOfPoses2DInf>*
createEdgeRegistrationDecider() {
	return new T;
}
template<typename T>
mrpt::graphslam::optimizers::CGraphSlamOptimizer<mrpt::graphs::CNetworkOfPoses2DInf>*
createGraphSlamOptimizer() {
	return new T;
}
/**}*/

} } } // END OF NAMESPACES

/**\}*/
