/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "graphslam-precomp.h"  // Precompiled headers
#include <mrpt/graphslam/apps_related/TUserOptionsChecker.h>

namespace mrpt { namespace graphslam { namespace supplementary {

//////////////////////////////////////////////////////////////
TUserOptionsChecker::TUserOptionsChecker():
	sep_header(40, '='),
	sep_subheader(20, '-')
{
	MRPT_START;

	this->createDeciderOptimizerMappings();
	this->populateDeciderOptimizerProperties();

	MRPT_END;
}
TUserOptionsChecker::~TUserOptionsChecker(){
	using namespace std;

	// release the instances holding the descriptions of the available
	// deciders/optimizers
	for (vector<TRegistrationDeciderProps*>::iterator
			it = regs_descriptions.begin();
			it != regs_descriptions.end();
			++it) {
		delete *it;
	}
	for (vector<TOptimizerProps*>::iterator
			it = optimizers_descriptions.begin();
			it != optimizers_descriptions.end();
			++it) {
		delete *it;
	}

}

void TUserOptionsChecker::createDeciderOptimizerMappings() {
	MRPT_START;

	using namespace mrpt::graphslam::deciders;
	using namespace mrpt::graphslam::optimizers;
	using namespace mrpt::graphs;

	// node registration deciders
	node_regs_map["CFixedIntervalsNRD"] =
		&createNodeRegistrationDecider<CFixedIntervalsNRD<CNetworkOfPoses2DInf> >;
	node_regs_map["CEmptyNRD"] =
		&createNodeRegistrationDecider<CEmptyNRD<CNetworkOfPoses2DInf> >;
	node_regs_map["CICPCriteriaNRD"] =
		&createNodeRegistrationDecider<CICPCriteriaNRD<CNetworkOfPoses2DInf> >;

	// edge registration deciders
	edge_regs_map["CICPCriteriaERD"] =
		&createEdgeRegistrationDecider<CICPCriteriaERD<CNetworkOfPoses2DInf> >;
	edge_regs_map["CEmptyERD"] =
		&createEdgeRegistrationDecider<CEmptyERD<CNetworkOfPoses2DInf> >;
	edge_regs_map["CLoopCloserERD"] =
		&createEdgeRegistrationDecider<CLoopCloserERD<CNetworkOfPoses2DInf> >;

	// optimizers
	optimizers_map["CLevMarqGSO"] =
		&createGraphSlamOptimizer<CLevMarqGSO<CNetworkOfPoses2DInf> >;

	MRPT_END;
}

//////////////////////////////////////////////////////////////
void TUserOptionsChecker::dumpRegistrarsToConsole(
		std::string reg_type /* = "all" */) const {
	MRPT_START;
	using namespace std;
	using  namespace mrpt;

	ASSERTMSG_((system::strCmpI(reg_type, "node") ||
				system::strCmpI(reg_type, "edge") ||
				system::strCmpI(reg_type, "all")),
			format("Registrar string '%s' does not match a known registrar name.\n"
				"Specify 'node' 'edge' or 'all'", reg_type.c_str()));


	if ( system::strCmpI(reg_type, "node") || system::strCmpI(reg_type, "edge") ) {

		cout << endl << "Available " << system::upperCase(reg_type) << " Registration Deciders: " << endl;
		cout << sep_header << endl;
		for (vector<TRegistrationDeciderProps*>::const_iterator dec_it = regs_descriptions.begin();
				dec_it != regs_descriptions.end(); ++dec_it) {
			TRegistrationDeciderProps* dec = *dec_it;
			if ( system::strCmpI(dec->type, reg_type) ) {
				cout << dec->name << endl;
				cout << sep_subheader << endl;
				cout << "\t- " << "Description: " <<  dec->description << endl;
				cout << "\t- " << "Rawlog Format: " <<  dec->rawlog_format << endl;
				cout << "\t- " << "Observations that can be used: " << endl;
				for (vector<string>::const_iterator obs_it = dec->observations_used.begin();
						obs_it != dec->observations_used.end(); ++obs_it) {
					cout << "\t\t+ " << *obs_it << endl;
				}
			}
		}
	}
	else { // print both
		dumpRegistrarsToConsole("node");
		dumpRegistrarsToConsole("edge");
	}

	MRPT_END;
}

//////////////////////////////////////////////////////////////
void TUserOptionsChecker::dumpOptimizersToConsole() const {
	MRPT_START;

	using namespace std;

	cout << endl << "Available GraphSlam Optimizer classes: " << endl;
	cout << sep_header << endl;

	for (vector<TOptimizerProps*>::const_iterator opt_it = optimizers_descriptions.begin();
			opt_it != optimizers_descriptions.end(); ++opt_it) {
		TOptimizerProps* opt = *opt_it;
		cout << opt->name << endl;
		cout << sep_subheader << endl;
		cout << "\t- " << "Description: " <<  opt->description << endl;
	}

	MRPT_END;
}

//////////////////////////////////////////////////////////////
bool TUserOptionsChecker::checkRegistrationDeciderExists(
		std::string given_reg,
		std::string reg_type) const {
	MRPT_START;

	using namespace std;
	using  namespace mrpt;

	ASSERTMSG_((system::strCmpI(reg_type, "node") ||
			system::strCmpI(reg_type, "edge")),
			format("Registrar string '%s' does not match a known registrar name.\n"
				"Specify 'node' or 'edge' ", reg_type.c_str()));
	bool found = false;

	for (vector<TRegistrationDeciderProps*>::const_iterator
			dec_it = regs_descriptions.begin();
			dec_it != regs_descriptions.end(); ++dec_it) {
		TRegistrationDeciderProps* dec = *dec_it;
		if (system::strCmpI(dec->type, reg_type)) {
			if (system::strCmpI(dec->name, given_reg)) {
				found = true;
				return found;
			}
		}
	}

	return found;
	MRPT_END;
}

//////////////////////////////////////////////////////////////
bool TUserOptionsChecker::checkOptimizerExists(std::string given_opt) const {
	MRPT_START;
	using namespace std;
	using  namespace mrpt;

	bool found = false;

	for (vector<TOptimizerProps*>::const_iterator
			opt_it = optimizers_descriptions.begin();
			opt_it != optimizers_descriptions.end(); ++opt_it) {
		TOptimizerProps* opt = *opt_it;
		if ( system::strCmpI(opt->name, given_opt) ) {
			found = true;
			return found;
		}
	}

	return found;
	MRPT_END;
}
//////////////////////////////////////////////////////////////

void TUserOptionsChecker::populateDeciderOptimizerProperties() {
	MRPT_START;

	// reset the vectors - in case they contain any elements
	regs_descriptions.clear();
	optimizers_descriptions.clear();

	// registering the available deciders
	{ // CFixedIntervalsNRD
		TRegistrationDeciderProps* dec = new TRegistrationDeciderProps;
		dec->name = "CFixedIntervalsNRD";
		dec->description = "Register a new node if the distance from the previous node surpasses a predefined distance threshold. Uses odometry information for estimating the robot movement";
		dec->type = "Node";
		dec->rawlog_format = "Both";
		dec->observations_used.push_back("CActionRobotMovement2D - Format #1");
		dec->observations_used.push_back("CObservationOdometry - Format #2");

		regs_descriptions.push_back(dec);
	}
	{ // CICPCriteriaNRD
		TRegistrationDeciderProps* dec = new TRegistrationDeciderProps;
		dec->name = "CICPCriteriaNRD";
		dec->description = "Register a new node if the distance from the previous node surpasses a predefined distance threshold. Uses 2D/3D RangeScans alignment for estimating the robot movement";
		dec->type = "Node";
		dec->rawlog_format = "#2 - Observation-only";
		dec->observations_used.push_back("CObservation2DRangeScan - Format #2");
		dec->observations_used.push_back("CObservation3DRangeScan - Format #2");

		regs_descriptions.push_back(dec);
	}
	{ // CEmptyNRD
		TRegistrationDeciderProps* dec = new TRegistrationDeciderProps;
		dec->name = "CEmptyNRD";
		dec->description = "Empty Decider - does nothing when its class methods are called";
		dec->type = "Node";
		dec->rawlog_format = "Both";

		regs_descriptions.push_back(dec);
	}
	{ // CICPCriteriaERD
		TRegistrationDeciderProps* dec = new TRegistrationDeciderProps;
		dec->name = "CICPCriteriaERD";
		dec->description = "Register a new edge by alligning the provided 2D/3D RangeScans of 2 nodes. Uses the goodness of the ICP Alignment as the criterium for adding a new edge";
		dec->type = "Edge";
		dec->rawlog_format = "Both";
		dec->observations_used.push_back("CObservation2DRangeScan - Format #1, #2");
		dec->observations_used.push_back("CObservation3DRangeScan - Format #2");

		regs_descriptions.push_back(dec);
	}
	{ // CEmptyERD
		TRegistrationDeciderProps* dec = new TRegistrationDeciderProps;
		dec->name = "CEmptyERD";
		dec->description = "Empty Decider - does nothing when its class methods are called";
		dec->type = "Edge";
		dec->rawlog_format = "Both";

		regs_descriptions.push_back(dec);
	}
	{ // CLoopCloserERD
		TRegistrationDeciderProps* dec = new TRegistrationDeciderProps;
		dec->name = "CLoopCloserERD";
		dec->description = "Partition the map and register *sets* of edges based on the Pairwise consistency matrix of each set.";
		dec->type = "Edge";
		dec->rawlog_format = "Both";
		dec->observations_used.push_back("CObservation2DRangeScan - Format #1, #2");

		regs_descriptions.push_back(dec);
	}

	// registering the available optimizers
	{ // CLevMarqGSO
		TOptimizerProps* opt = new TOptimizerProps;
		opt->name = "CLevMarqGSO";
		opt->description = "Levenberg-Marqurdt non-linear graphSLAM solver";

		optimizers_descriptions.push_back(opt);
	}
	MRPT_END
}

} } } // END OF NAMESPACES
