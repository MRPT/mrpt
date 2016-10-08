/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "graphslam-precomp.h"  // Precompiled headers
#include <mrpt/graphslam/supplementary_funs.h>

// Implementation for for mrpt-graphslam supplementary functions
namespace mrpt { namespace graphslam { namespace supplementary {

std::string sep_header(40, '=');
std::string sep_subheader(20, '-');

//////////////////////////////////////////////////////////////
void dumpRegistrarsToConsole(
		const std::vector<TRegistrationDeciderProps*>& registrars_vec,
		std::string reg_type /* = "all" */) {
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
		for (vector<TRegistrationDeciderProps*>::const_iterator dec_it = registrars_vec.begin();
				dec_it != registrars_vec.end(); ++dec_it) {
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
		dumpRegistrarsToConsole(registrars_vec, "node");
		dumpRegistrarsToConsole(registrars_vec, "edge");
	}

	MRPT_END;
}

//////////////////////////////////////////////////////////////
void dumpOptimizersToConsole(
		const std::vector<TOptimizerProps*>& optimizers_vec) {
	MRPT_START;

	using namespace std;

	cout << endl << "Available GraphSlam Optimizer classes: " << endl;
	cout << sep_header << endl;

	for (vector<TOptimizerProps*>::const_iterator opt_it = optimizers_vec.begin();
			opt_it != optimizers_vec.end(); ++opt_it) {
		TOptimizerProps* opt = *opt_it;
		cout << opt->name << endl;
		cout << sep_subheader << endl;
		cout << "\t- " << "Description: " <<  opt->description << endl;
	}

	MRPT_END;
}

//////////////////////////////////////////////////////////////
bool checkRegistrationDeciderExists(
		const std::vector<TRegistrationDeciderProps*>& registrars_vec,
		std::string given_reg,
		std::string reg_type) {
	MRPT_START;

	using namespace std;
	using  namespace mrpt;

	ASSERTMSG_((system::strCmpI(reg_type, "node") ||
			system::strCmpI(reg_type, "edge")),
			format("Registrar string '%s' does not match a known registrar name.\n"
				"Specify 'node' or 'edge' ", reg_type.c_str()));
	bool found = false;

	for (vector<TRegistrationDeciderProps*>::const_iterator
			dec_it = registrars_vec.begin();
			dec_it != registrars_vec.end(); ++dec_it) {
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
bool checkOptimizerExists(
		const std::vector<TOptimizerProps*>& optimizers_vec,
		std::string given_opt) {
	MRPT_START;
	using namespace std;
	using  namespace mrpt;

	bool found = false;

	for (vector<TOptimizerProps*>::const_iterator
			opt_it = optimizers_vec.begin();
			opt_it != optimizers_vec.end(); ++opt_it) {
		TOptimizerProps* opt = *opt_it;
		if ( system::strCmpI(opt->name, given_opt) ) {
			found = true;
			return found;
		}
	}

	return found;
	MRPT_END;
}

} } } // END OF NAMESPACES
