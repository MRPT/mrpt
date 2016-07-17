/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CICPGOODNESSNRD_IMPL_H
#define CICPGOODNESSNRD_IMPL_H

using namespace mrpt::graphslam::deciders;

// Ctors, Dtors
//////////////////////////////////////////////////////////////

template<class GRAPH_t>
CICPGoodnessNRD_t<GRAPH_t>::CICPGoodnessNRD_t():
	params(*this) // pass reference to self when initializing the parameters
{
	this->initCICPGoodnessNRD_t();
}
template<class GRAPH_t>
void CICPGoodnessNRD_t<GRAPH_t>::initCICPGoodnessNRD_t() {

	m_first_time_call2D = true;
	m_first_time_call3D = true;
	m_is_using_3DScan = false;

	m_graph = NULL;

	// Current node registration decider *decides* how many nodes are there
	// currently in the graph (no need to ask m_graph->nodeCount)..
	m_nodeID_max  = INVALID_NODEID;

	m_curr_timestamp = INVALID_TIMESTAMP;
	m_prev_timestamp = INVALID_TIMESTAMP;

	// I am sure of the initial position, set to identity matrix
	double tmp[] = {
		1.0, 0.0, 0.0,
		0.0, 1.0 ,0.0,
		0.0, 0.0, 0.0 };
	InfMat init_path_uncertainty(tmp);
	m_since_prev_node_PDF.cov_inv = init_path_uncertainty;
	m_since_prev_node_PDF.mean = pose_t();

	m_out_logger.setName("CICPGoodnessNRD");
	m_out_logger.setLoggingLevel(LVL_DEBUG);

	m_out_logger.log("Initialized class object", LVL_DEBUG);
}
template<class GRAPH_t>
CICPGoodnessNRD_t<GRAPH_t>::~CICPGoodnessNRD_t() { }

template<class GRAPH_t>
bool CICPGoodnessNRD_t<GRAPH_t>::updateState(
		mrpt::obs::CActionCollectionPtr action,
		mrpt::obs::CSensoryFramePtr observations,
		mrpt::obs::CObservationPtr observation )  {
	MRPT_START;
	MRPT_UNUSED_PARAM(action);
	m_time_logger.enter("CICPGoodnessNRD::updateState");
	bool registered_new_node = false;

	if (observation.present()) { // Observation-Only Rawlog
		// delegate the action to the method responsible
		if (IS_CLASS(observation, CObservation2DRangeScan) ) { // 2D
			mrpt::obs::CObservation2DRangeScanPtr curr_laser_scan = 
				static_cast<mrpt::obs::CObservation2DRangeScanPtr>(observation);
			registered_new_node = updateState2D(curr_laser_scan);

		}
		if (IS_CLASS(observation, CObservation3DRangeScan) ) { // 3D
			mrpt::obs::CObservation3DRangeScanPtr curr_laser_scan = 
				static_cast<mrpt::obs::CObservation3DRangeScanPtr>(observation);
			registered_new_node = updateState3D(curr_laser_scan);
		}
	}
	else { // action-observations rawlog
		if (observations->getObservationByClass<CObservation2DRangeScan>()) { // 2D
			CObservation2DRangeScanPtr curr_laser_scan = 
				observations->getObservationByClass<CObservation2DRangeScan>();
			registered_new_node = updateState2D(curr_laser_scan);
		}
		else if (observations->getObservationByClass<CObservation3DRangeScan>()){	// 3D - EXPERIMENTAL, has not been tested
			CObservation3DRangeScanPtr curr_laser_scan = 
				observations->getObservationByClass<CObservation3DRangeScan>();
			registered_new_node = updateState3D(curr_laser_scan);
		}
	}

	// reset the constraint since the last registered node
	if (registered_new_node) {
		m_since_prev_node_PDF = constraint_t();
	}

	// TODO - implement checkIfInvalidDataset
	m_time_logger.leave("CICPGoodnessNRD::updateState");
	return registered_new_node;

	MRPT_END;
}

template<class GRAPH_t>
bool CICPGoodnessNRD_t<GRAPH_t>::updateState2D(
		mrpt::obs::CObservation2DRangeScanPtr scan2d) {
	MRPT_START;
	bool registered_new_node = false;

	m_curr_laser_scan2D = scan2d;
	if (m_last_laser_scan2D.null()) {
		m_out_logger.log("First time call for updateState2D.", LVL_WARN);
		// initialize the last_laser_scan here - afterwards updated inside the
		// checkRegistrationCondition*D method
		m_last_laser_scan2D = m_curr_laser_scan2D;
	}
	else {
		registered_new_node = checkRegistrationCondition2D(); 
	}

	return registered_new_node;
	MRPT_END;
}
template<class GRAPH_t>
bool CICPGoodnessNRD_t<GRAPH_t>::checkRegistrationCondition2D() {
	MRPT_START;
	bool registered_new_node = false;
	m_out_logger.log("In checkRegistrationCondition2D..");

	constraint_t rel_edge;
	mrpt::slam::CICP::TReturnInfo icp_info;

	this->getICPEdge(
			*m_last_laser_scan2D,
			*m_curr_laser_scan2D,
			&rel_edge,
			NULL,
			&icp_info);
	// append current ICP edge to the sliding window
	m_ICP_sliding_win.addNewMeasurement(icp_info.goodness);

	m_out_logger.logFmt("Current ICP constraint: \n\tEdge: %s\n\tNorm: %f", 
				rel_edge.getMeanVal().asString().c_str(), 
				rel_edge.getMeanVal().norm());

	// Criterions for updating PDF since last registered node
	// - ICP goodness > threshold goodness
	if (m_ICP_sliding_win.evaluateICPgoodness(icp_info.goodness) ) {
		m_since_prev_node_PDF += rel_edge;
		m_last_laser_scan2D = m_curr_laser_scan2D;
		registered_new_node = this->checkRegistrationCondition();
	}


	return registered_new_node;
	MRPT_END;
}
template<class GRAPH_t>
bool CICPGoodnessNRD_t<GRAPH_t>::updateState3D(
		mrpt::obs::CObservation3DRangeScanPtr scan3d) {
	MRPT_START;
	bool registered_new_node = false;

	m_curr_laser_scan3D = scan3d;
	m_curr_laser_scan3D->load();
	m_curr_laser_scan3D->project3DPointsFromDepthImage();

	if (m_last_laser_scan3D.null()) {
		m_out_logger.log("First time call for updateState3D.", LVL_WARN);
		// initialize the last_laser_scan here - afterwards updated inside the
		// checkRegistrationCondition*D method
		m_last_laser_scan3D = m_curr_laser_scan3D;
	}
	else {
		registered_new_node = checkRegistrationCondition3D(); 
	}

	return registered_new_node;
	MRPT_END;
}

template<class GRAPH_t>
bool CICPGoodnessNRD_t<GRAPH_t>::checkRegistrationCondition3D() {
	MRPT_START;
	bool registered_new_node = false;

	m_out_logger.log("In checkRegistrationCondition3D..");

	constraint_t* rel_edge = new constraint_t;
	mrpt::slam::CICP::TReturnInfo icp_info;

	this->getICPEdge(
			*m_last_laser_scan3D,
			*m_curr_laser_scan3D,
			rel_edge,
			NULL,
			&icp_info);
	// append current ICP edge to the sliding window
	m_ICP_sliding_win.addNewMeasurement(icp_info.goodness);

	// Criterions for updating PDF since last registered node
	// - ICP goodness > threshold goodness
	// - Small Z displacement

	if (!rel_edge) {
		m_out_logger.logFmt("NULL rel_edge from getICPEdge procedure");
		return false;
	}

	m_out_logger.logFmt("Current ICP constraint: \n\tEdge: %s\n\tNorm: %f", 
				rel_edge->getMeanVal().asString().c_str(), 
				rel_edge->getMeanVal().norm());
	m_out_logger.logFmt("ICP Alignment operation:\
			\n\tnIterations: %d\
			\n\tquality: %f\
			\n\tgoodness: %.f\n",
			icp_info.nIterations, icp_info.quality, icp_info.goodness);

	if (m_ICP_sliding_win.evaluateICPgoodness(icp_info.goodness) ) {
		m_out_logger.logFmt("Using the above constraint...");
		m_since_prev_node_PDF += *rel_edge;
		m_last_laser_scan3D = m_curr_laser_scan3D;
		registered_new_node = this->checkRegistrationCondition();
	}

	return registered_new_node;
	MRPT_END;
}

template<class GRAPH_t>
bool CICPGoodnessNRD_t<GRAPH_t>::checkRegistrationCondition() {
	MRPT_START;
	bool registered_new_node = false;
	m_out_logger.log("In checkRegistrationCondition");

	// Criterions for adding a new node
	// - Covered distance since last node > registration_max_distance
	// - Angle difference since last node > registration_max_angle

	bool angle_crit = false;
	if (m_use_angle_difference_node_reg) {
		angle_crit = fabs(wrapToPi(m_since_prev_node_PDF.getMeanVal().phi())) >
			params.registration_max_angle;
	}
	bool distance_crit = false;
	if (m_use_distance_node_reg) {
		distance_crit =
			m_since_prev_node_PDF.getMeanVal().norm() > params.registration_max_distance;
	}

	// actual check
	if (distance_crit || angle_crit) {
		registered_new_node = true;
		this->registerNewNode();
	}

	return registered_new_node;
	MRPT_END;
}

template<class GRAPH_t>
void CICPGoodnessNRD_t<GRAPH_t>::registerNewNode() {
	MRPT_START;

	mrpt::utils::TNodeID from = m_nodeID_max;
	mrpt::utils::TNodeID to = ++m_nodeID_max;

	m_graph->nodes[to] = m_graph->nodes[from] + m_since_prev_node_PDF.getMeanVal();
  m_graph->insertEdgeAtEnd(from, to, m_since_prev_node_PDF);

	m_out_logger.logFmt("Registered new node:\n\t%lu => %lu\n\tEdge: %s",
				from, to, m_since_prev_node_PDF.getMeanVal().asString().c_str());

	MRPT_END;
}

template<class GRAPH_t>
void CICPGoodnessNRD_t<GRAPH_t>::setGraphPtr(GRAPH_t* graph) {
	m_graph = graph;

	// get the last registrered node + corresponding pose - root
	m_nodeID_max = m_graph->root;

	m_out_logger.log("Fetched the graph successfully", LVL_DEBUG);
}
template<class GRAPH_t>
void CICPGoodnessNRD_t<GRAPH_t>::loadParams(const std::string& source_fname) {
	MRPT_START;

	params.loadFromConfigFileName(source_fname,
			"NodeRegistrationDeciderParameters");
	m_ICP_sliding_win.loadFromConfigFileName(source_fname,
			"NodeRegistrationDeciderParameters");

	// set the logging level if given by the user
	CConfigFile source(source_fname);
	// Minimum verbosity level of the logger
	int min_verbosity_level = source.read_int(
			"NodeRegistrationDeciderParameters",
			"class_verbosity",
			1, false);
	m_out_logger.setMinLoggingLevel(VerbosityLevel(min_verbosity_level));

	m_out_logger.log("Successfully loaded parameters.", LVL_DEBUG);
	MRPT_END;
}
template<class GRAPH_t>
void CICPGoodnessNRD_t<GRAPH_t>::printParams() const {
	MRPT_START;

	params.dumpToConsole();
	m_ICP_sliding_win.dumpToConsole();

	MRPT_END;
}

template<class GRAPH_t>
void CICPGoodnessNRD_t<GRAPH_t>::getDescriptiveReport(std::string* report_str) const {
	MRPT_START;

	const std::string report_sep(2, '\n');
	const std::string header_sep(80, '#');

	// Report on graph
	stringstream class_props_ss;
	class_props_ss << "ICP Goodness-based Registration Procedure Summary: " << std::endl;
	class_props_ss << header_sep << std::endl;

	// time and output logging
	const std::string time_res = m_time_logger.getStatsAsText();
	const std::string output_res = m_out_logger.getAsString();

	// merge the individual reports
	report_str->clear();

	*report_str += class_props_ss.str();
	*report_str += report_sep;

	// loggers results
	*report_str += time_res;
	*report_str += report_sep;

	*report_str += output_res;
	*report_str += report_sep;

	MRPT_END;
}


// TParams
//////////////////////////////////////////////////////////////
template<class GRAPH_t>
CICPGoodnessNRD_t<GRAPH_t>::TParams::TParams(decider_t& d):
	decider(d)
{ }
template<class GRAPH_t>
CICPGoodnessNRD_t<GRAPH_t>::TParams::~TParams() { }
template<class GRAPH_t>
void CICPGoodnessNRD_t<GRAPH_t>::TParams::dumpToTextStream(
		mrpt::utils::CStream &out) const {
	MRPT_START;

	out.printf("------------------[ ICP Fixed Intervals Node Registration ]------------------\n");
	out.printf("Max distance for registration = %.2f m\n",
			registration_max_distance);
	out.printf("Max Angle for registration    = %.2f deg\n",
			RAD2DEG(registration_max_angle));

	decider.range_scanner_t::params.dumpToTextStream(out);

	MRPT_END;
}
template<class GRAPH_t>
void CICPGoodnessNRD_t<GRAPH_t>::TParams::loadFromConfigFile(
		const mrpt::utils::CConfigFileBase &source,
    const std::string &section) {
  MRPT_START;

	registration_max_distance = source.read_double( section,
			"registration_max_distance",
			0.5 /* meter */, false);
	registration_max_angle = source.read_double( section,
			"registration_max_angle",
			10 /* degrees */, false);
	registration_max_angle = DEG2RAD(registration_max_angle);

	// load the icp parameters - from "ICP" section explicitly
	decider.range_scanner_t::params.loadFromConfigFile(source, "ICP");

	MRPT_END;
}


#endif /* end of include guard: CICPGOODNESSNRD_IMPL_H */
