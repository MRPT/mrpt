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

// Properties struct for the Registration Decider Classes
struct TRegistrationDeciderProps {
	TRegistrationDeciderProps() {}
	~TRegistrationDeciderProps() {}

  std::string name;
  std::string description;
  std::string type; // type of registration decider - node/edge?
  std::string rawlog_format; // rawlog formats that the decider can be used in
  std::vector<std::string> observations_used;
};

// Properties struct for the Optimizer Classes
struct TOptimizerProps {
	TOptimizerProps() {}
	~TOptimizerProps() {}

	std::string name;
	std::string description;

};

// Check if the given registration decider is implemented
bool checkRegistrationDeciderExists(
		const std::vector<TRegistrationDeciderProps*>& registrars_vec,
		std::string given_reg,
		std::string reg_type);

// Check if the given optimizer is implemented
bool checkOptimizerExists(
		const std::vector<TOptimizerProps*>& optimizers_vec,
		std::string given_opt);

// Print the properties of the available deciders
void dumpRegistrarsToConsole(
		const std::vector<TRegistrationDeciderProps*>& registrars_vec,
		std::string reg_type="all"
		);

// Print the properties of the available optimizers
void dumpOptimizersToConsole(
		const std::vector<TOptimizerProps*>& optimizers_vec);

// functions for initializing decider/optimizer instances based on the user
// command line choices - http://stackoverflow.com/a/582456/2843583
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

