/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

// implementation of the TUserOptionsChecker class template methods
namespace mrpt::graphslam::apps
{
template <class GRAPH_t>
TUserOptionsChecker<GRAPH_t>::TUserOptionsChecker()
	: sep_header(40, '='), sep_subheader(20, '-')
{
}

template <class GRAPH_t>
TUserOptionsChecker<GRAPH_t>::~TUserOptionsChecker()
{
	using namespace std;

	// release the instances holding the descriptions of the available
	// deciders/optimizers
	for (auto it = regs_descriptions.begin(); it != regs_descriptions.end();
		 ++it)
	{
		delete *it;
	}
	for (auto it = optimizers_descriptions.begin();
		 it != optimizers_descriptions.end(); ++it)
	{
		delete *it;
	}
}

template <class GRAPH_t>
void TUserOptionsChecker<GRAPH_t>::createDeciderOptimizerMappings()
{
	MRPT_START;

	using namespace mrpt::graphslam::deciders;
	using namespace mrpt::graphslam::optimizers;
	using namespace mrpt::graphs;
	using namespace mrpt::poses;

	// node registration deciders
	node_regs_map["CEmptyNRD"] =
		&createNodeRegistrationDecider<CEmptyNRD<GRAPH_t>>;
	node_regs_map["CFixedIntervalsNRD"] =
		&createNodeRegistrationDecider<CFixedIntervalsNRD<GRAPH_t>>;
	// edge registration deciders
	edge_regs_map["CEmptyERD"] =
		&createEdgeRegistrationDecider<CEmptyERD<GRAPH_t>>;
	// optimizers
	optimizers_map["CLevMarqGSO"] =
		&createGraphSlamOptimizer<CLevMarqGSO<GRAPH_t>>;
	optimizers_map["CEmptyGSO"] =
		&createGraphSlamOptimizer<CLevMarqGSO<GRAPH_t>>;

	// create the decider optimizer, specific to the GRAPH_T template type
	this->_createDeciderOptimizerMappings();

	MRPT_END;
}

template <class GRAPH_t>
void TUserOptionsChecker<GRAPH_t>::_createDeciderOptimizerMappings()
{
}

// deciders/optpimizers specific to the 2D SLAM cases
template <>
inline void TUserOptionsChecker<
	mrpt::graphs::CNetworkOfPoses2DInf>::_createDeciderOptimizerMappings()
{
	using namespace mrpt::graphs;

	node_regs_map["CICPCriteriaNRD"] =
		&createNodeRegistrationDecider<CICPCriteriaNRD<CNetworkOfPoses2DInf>>;
	edge_regs_map["CICPCriteriaERD"] =
		&createEdgeRegistrationDecider<CICPCriteriaERD<CNetworkOfPoses2DInf>>;
	edge_regs_map["CLoopCloserERD"] =
		&createEdgeRegistrationDecider<CLoopCloserERD<CNetworkOfPoses2DInf>>;
}
template <>
inline void TUserOptionsChecker<
	mrpt::graphs::CNetworkOfPoses2DInf_NA>::_createDeciderOptimizerMappings()
{
	using namespace mrpt::graphs;

	node_regs_map["CICPCriteriaNRD"] = &createNodeRegistrationDecider<
		CICPCriteriaNRD<CNetworkOfPoses2DInf_NA>>;
	edge_regs_map["CICPCriteriaERD"] = &createEdgeRegistrationDecider<
		CICPCriteriaERD<CNetworkOfPoses2DInf_NA>>;
	edge_regs_map["CLoopCloserERD"] =
		&createEdgeRegistrationDecider<CLoopCloserERD<CNetworkOfPoses2DInf_NA>>;
}

// deciders/optpimizers specific to the 3D SLAM cases
template <>
inline void TUserOptionsChecker<
	mrpt::graphs::CNetworkOfPoses3DInf>::_createDeciderOptimizerMappings()
{
}

template <class GRAPH_t>
void TUserOptionsChecker<GRAPH_t>::dumpRegistrarsToConsole(
	std::string reg_type /*="all"*/) const
{
	MRPT_START;
	using namespace std;
	using namespace mrpt;

	ASSERTDEBMSG_(
		(system::strCmpI(reg_type, "node") ||
		 system::strCmpI(reg_type, "edge") || system::strCmpI(reg_type, "all")),
		format(
			"Registrar string '%s' does not match a known registrar name.\n"
			"Specify 'node' 'edge' or 'all'",
			reg_type.c_str()));

	if (system::strCmpI(reg_type, "node") || system::strCmpI(reg_type, "edge"))
	{
		cout << endl
			 << "Available " << system::upperCase(reg_type)
			 << " Registration Deciders: " << endl;
		cout << sep_header << endl;

		for (auto dec_it = regs_descriptions.begin();
			 dec_it != regs_descriptions.end(); ++dec_it)
		{
			TRegistrationDeciderProps* dec = *dec_it;
			if (system::strCmpI(dec->type, reg_type))
			{
				cout << dec->name << endl;
				cout << sep_subheader << endl;
				cout << "\t- "
					 << "Description: " << dec->description << endl;
				cout << "\t- "
					 << "Rawlog Format: " << dec->rawlog_format << endl;
				cout << "\t- "
					 << "Observations that can be used: " << endl;
				cout << "\t- "
					 << "Multi-robot SLAM capable decider: "
					 << (dec->is_mr_slam_class ? "TRUE" : "FALSE") << endl;
				cout << "\t- "
					 << "SLAM Type: " << endl;
				if (dec->is_slam_2d)
				{
					cout << "\t\t+ "
						 << "2D" << endl;
				}
				if (dec->is_slam_3d)
				{
					cout << "\t\t+ "
						 << "3D" << endl;
				}
				cout << endl;
				for (auto obs_it = dec->observations_used.begin();
					 obs_it != dec->observations_used.end(); ++obs_it)
				{
					cout << "\t\t+ " << *obs_it << endl;
				}
			}
		}
	}
	else
	{  // print both
		dumpRegistrarsToConsole("node");
		dumpRegistrarsToConsole("edge");
	}

	MRPT_END;
}

template <class GRAPH_t>
void TUserOptionsChecker<GRAPH_t>::dumpOptimizersToConsole() const
{
	MRPT_START;

	using namespace std;

	cout << endl << "Available GraphSlam Optimizer classes: " << endl;
	cout << sep_header << endl;

	for (auto opt_it = optimizers_descriptions.begin();
		 opt_it != optimizers_descriptions.end(); ++opt_it)
	{
		TOptimizerProps* opt = *opt_it;
		cout << opt->name << endl;
		cout << sep_subheader << endl;
		cout << "\t- "
			 << "Description: " << opt->description << endl;

		cout << "\t- "
			 << "Multi-robot SLAM capable optimizer: "
			 << (opt->is_mr_slam_class ? "TRUE" : "FALSE");
		cout << "\t- "
			 << "SLAM Type: " << endl;
		if (opt->is_slam_2d)
		{
			cout << "\t\t+ "
				 << "2D" << endl;
		}
		if (opt->is_slam_3d)
		{
			cout << "\t\t+ "
				 << "3D" << endl;
		}
	}
	MRPT_END;
}

template <class GRAPH_t>
bool TUserOptionsChecker<GRAPH_t>::checkRegistrationDeciderExists(
	std::string given_reg, std::string reg_type) const
{
	MRPT_START;

	using namespace std;
	using namespace mrpt;
	using namespace mrpt::poses;

	ASSERTDEBMSG_(
		(system::strCmpI(reg_type, "node") ||
		 system::strCmpI(reg_type, "edge")),
		format(
			"Registrar string \"%s\" does not match a known registrar name.\n"
			"Specify 'node' or 'edge' ",
			reg_type.c_str()));
	bool found = false;

	for (auto dec_it = regs_descriptions.begin();
		 dec_it != regs_descriptions.end(); ++dec_it)
	{
		TRegistrationDeciderProps* dec = *dec_it;

		// TODO - check this
		// if decider is not suitable for the selected SLAM type, ignore.
		pose_t p;
		if ((!dec->is_slam_2d && IS_CLASS(&p, CPose2D)) ||
			(!dec->is_slam_3d && IS_CLASS(&p, CPose3D)))
		{
			continue;
		}

		if (system::strCmpI(dec->type, reg_type))
		{
			if (system::strCmpI(dec->name, given_reg))
			{
				found = true;
				return found;
			}
		}
	}

	return found;
	MRPT_END;
}

template <class GRAPH_t>
bool TUserOptionsChecker<GRAPH_t>::checkOptimizerExists(
	std::string given_opt) const
{
	MRPT_START;
	using namespace std;
	using namespace mrpt;
	using namespace mrpt::poses;

	bool found = false;

	for (auto opt_it = optimizers_descriptions.begin();
		 opt_it != optimizers_descriptions.end(); ++opt_it)
	{
		TOptimizerProps* opt = *opt_it;

		// if optimizer is not suitable for the selected SLAM type, ignore.
		pose_t p;
		if ((!opt->is_slam_2d && IS_CLASS(&p, CPose2D)) &&
			(!opt->is_slam_3d && IS_CLASS(&p, CPose3D)))
		{
			continue;
		}

		if (system::strCmpI(opt->name, given_opt))
		{
			found = true;
			return found;
		}
	}

	return found;
	MRPT_END;
}

template <class GRAPH_t>
void TUserOptionsChecker<GRAPH_t>::populateDeciderOptimizerProperties()
{
	MRPT_START;

	// reset the vectors - in case they contain any elements
	regs_descriptions.clear();
	optimizers_descriptions.clear();

	// registering the available deciders
	{  // CFixedIntervalsNRD
		auto* dec = new TRegistrationDeciderProps;
		dec->name = "CFixedIntervalsNRD";
		dec->description =
			"Register a new node if the distance from the previous node "
			"surpasses a predefined distance threshold. Uses odometry "
			"information for estimating the robot movement";
		dec->type = "Node";
		dec->rawlog_format = "Both";
		dec->observations_used.emplace_back(
			"CActionRobotMovement2D - Format #1");
		dec->observations_used.emplace_back("CObservationOdometry - Format #2");
		dec->is_slam_2d = true;
		dec->is_slam_3d = true;

		regs_descriptions.push_back(dec);
	}
	{  // CICPCriteriaNRD
		auto* dec = new TRegistrationDeciderProps;
		dec->name = "CICPCriteriaNRD";
		dec->description =
			"Register a new node if the distance from the previous node "
			"surpasses a predefined distance threshold. Uses 2D/3D RangeScans "
			"alignment for estimating the robot movement";
		dec->type = "Node";
		dec->rawlog_format = "#2 - Observation-only";
		dec->observations_used.emplace_back(
			"CObservation2DRangeScan - Format #2");
		dec->observations_used.emplace_back(
			"CObservation3DRangeScan - Format #2");
		dec->is_slam_2d = true;

		regs_descriptions.push_back(dec);
	}
	{  // CEmptyNRD
		auto* dec = new TRegistrationDeciderProps;
		dec->name = "CEmptyNRD";
		dec->description =
			"Empty Decider - does nothing when its class methods are called";
		dec->type = "Node";
		dec->rawlog_format = "Both";
		dec->is_mr_slam_class = true;
		dec->is_slam_2d = true;
		dec->is_slam_3d = true;

		regs_descriptions.push_back(dec);
	}
	{  // CICPCriteriaERD
		auto* dec = new TRegistrationDeciderProps;
		dec->name = "CICPCriteriaERD";
		dec->description =
			"Register a new edge by aligning the provided 2D/3D RangeScans of "
			"2 nodes. Uses the goodness of the ICP Alignment as the criterium "
			"for adding a new edge";
		dec->type = "Edge";
		dec->rawlog_format = "Both";
		dec->observations_used.emplace_back(
			"CObservation2DRangeScan - Format #1, #2");
		dec->observations_used.emplace_back(
			"CObservation3DRangeScan - Format #2");
		dec->is_slam_2d = true;

		regs_descriptions.push_back(dec);
	}
	{  // CEmptyERD
		auto* dec = new TRegistrationDeciderProps;
		dec->name = "CEmptyERD";
		dec->description =
			"Empty Decider - does nothing when its class methods are called";
		dec->type = "Edge";
		dec->rawlog_format = "Both";
		dec->is_mr_slam_class = true;
		dec->is_slam_2d = true;
		dec->is_slam_3d = true;

		regs_descriptions.push_back(dec);
	}
	{  // CLoopCloserERD
		auto* dec = new TRegistrationDeciderProps;
		dec->name = "CLoopCloserERD";
		dec->description =
			"Partition the map and register *sets* of edges based on the "
			"Pairwise consistency matrix of each set.";
		dec->type = "Edge";
		dec->rawlog_format = "Both";
		dec->observations_used.emplace_back(
			"CObservation2DRangeScan - Format #1, #2");
		dec->is_slam_2d = true;

		regs_descriptions.push_back(dec);
	}

	// registering the available optimizers
	{  // CEmptyGSO
		auto* opt = new TOptimizerProps;
		opt->name = "CEmptyGSO";
		opt->description =
			"Empty Optimizer - does nothing when its class methods are called";
		opt->is_mr_slam_class = true;
		opt->is_slam_2d = true;
		opt->is_slam_3d = true;

		optimizers_descriptions.push_back(opt);
	}

	{  // CLevMarqGSO
		auto* opt = new TOptimizerProps;
		opt->name = "CLevMarqGSO";
		opt->description = "Levenberg-Marqurdt non-linear graphSLAM solver";
		opt->is_mr_slam_class = true;
		opt->is_slam_2d = true;
		opt->is_slam_3d = true;

		optimizers_descriptions.push_back(opt);
	}

	MRPT_END
}
}  // namespace mrpt::graphslam::apps
