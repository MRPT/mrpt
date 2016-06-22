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
CICPGoodnessNRD_t<GRAPH_t>::CICPGoodnessNRD_t() {
	this->initCICPGoodnessNRD_t();
}
template<class GRAPH_t>
void CICPGoodnessNRD_t<GRAPH_t>::initCICPGoodnessNRD_t() {

	m_first_time_call = true;

	m_win = NULL;
	m_graph = NULL;

	// Current node registration decider *decides* how many nodes are there
	// currently in the graph (no need to ask m_graph->nodeCount..
	m_nodeID_max  = INVALID_NODEID;

	// I am sure of the initial position, set to identity matrix
	double tmp[] = {
		1.0, 0.0, 0.0,
		0.0, 1.0 ,0.0,
		0.0, 0.0, 0.0 };
	InfMat init_path_uncertainty(tmp);
	m_since_prev_node_PDF.cov_inv = init_path_uncertainty;
	m_since_prev_node_PDF.mean = pose_t();


	std::cout << "CICPGoodnessNRD: Initialized class object" << std::endl;
}
template<class GRAPH_t>
CICPGoodnessNRD_t<GRAPH_t>::~CICPGoodnessNRD_t() { }

template<class GRAPH_t>
bool CICPGoodnessNRD_t<GRAPH_t>::updateDeciderState(
		mrpt::obs::CActionCollectionPtr action,
		mrpt::obs::CSensoryFramePtr observations,
		mrpt::obs::CObservationPtr observation )  {
	MRPT_START;

	bool registered_new_node = false;

	MRPT_UNUSED_PARAM(action);
	MRPT_UNUSED_PARAM(observations);

	if (observation.present()) {
		if (IS_CLASS(observation, CObservation2DRangeScan) || 
				IS_CLASS(observation, CObservation3DRangeScan)) {
			// if 3D then just update the 2D counterpart
			if (IS_CLASS(observation, CObservation3DRangeScan)) {
				m_curr_laser_scan3D =
					static_cast<mrpt::obs::CObservation3DRangeScanPtr>(observation);
				// TODO deal with this..
				m_curr_laser_scan3D->load();
				this->convert3DTo2DRangeScan(
						/*from = */ m_curr_laser_scan3D,
						/*to   = */ &m_curr_laser_scan2D); 
			}
			else if (IS_CLASS(observation, CObservation2DRangeScan)) {
				m_curr_laser_scan2D = 
					static_cast<mrpt::obs::CObservation2DRangeScanPtr>(observation);
			}

			// if first_time in initialize the m_last_laser_scan as well
			if (m_first_time_call) {
				cout << "CICPGoodnessNRD: Registering first laser scan.." << endl;
				m_last_laser_scan2D = m_curr_laser_scan2D;
				m_first_time_call = false;
				return false;
			}
			registered_new_node = this->checkRegistrationCondition();
			//mrpt::system::pause();
		}
	}
	else { // FORMAT #1
		// TODO - implement this
	}

	if (registered_new_node) {
		// reset the relative PDF since the previous registered node
		// maybe put the covariance tmp here as well? 
		m_since_prev_node_PDF = constraint_t();
	}

	// TODO - implement checkIfInvalidDataset
	
	return registered_new_node;
	MRPT_END;
}

template<class GRAPH_t>
bool CICPGoodnessNRD_t<GRAPH_t>::checkRegistrationCondition() {
	MRPT_START;
	//cout << "CICPGoodnessNRD: In checkRegistrationCondition2D.." << endl;
	bool registered_new_node = false;

	constraint_t rel_edge;
	double goodness = this->getICPEdge(
			*m_last_laser_scan2D,
			*m_curr_laser_scan2D,
			&rel_edge );
	cout << "CICPGoodnessNRD: ICP goodness: " << goodness << endl;

	if (goodness > params.ICP_goodness_thresh) {
		m_since_prev_node_PDF += rel_edge;
		m_last_laser_scan2D = m_curr_laser_scan2D;

		cout << "CICPGoodnessNRD: norm = " << m_since_prev_node_PDF.getMeanVal().norm() << endl;
		cout << "CICPGoodnessNRD: angle = " << 
			RAD2DEG(fabs(wrapToPi(m_since_prev_node_PDF.getMeanVal().phi()))) << endl;
		// check if distance or angle difference is good enough for new node
		if ( m_since_prev_node_PDF.getMeanVal().norm() > params.registration_max_distance || 
				fabs(wrapToPi(m_since_prev_node_PDF.getMeanVal().phi())) > params.registration_max_angle ) {
			registered_new_node = true;
			this->registerNewNode();
		}
	}

	return registered_new_node;
	MRPT_END;
}

template<class GRAPH_t>
void CICPGoodnessNRD_t<GRAPH_t>::registerNewNode() {
	MRPT_START;

	mrpt::utils::TNodeID from = m_nodeID_max;
	mrpt::utils::TNodeID to = ++m_nodeID_max;

	std::cout << "CICPGoodnessNRD: Registering new node/edge: "
		<< from << " -> " << to << std::endl
		<< "\tedge: " << std::endl << m_since_prev_node_PDF << std::endl;

	m_graph->nodes[to] = m_graph->nodes[from] + m_since_prev_node_PDF.getMeanVal();
  m_graph->insertEdgeAtEnd(from, to, m_since_prev_node_PDF);

	MRPT_END;
}
template<class GRAPH_t>
void CICPGoodnessNRD_t<GRAPH_t>::setGraphPtr(GRAPH_t* graph) {
	m_graph = graph;

	// get the last registrered node + corresponding pose - root
	m_nodeID_max = m_graph->root;
	cout << "m_nodeID_max = " << m_nodeID_max << endl;

	std::cout << "CICPGoodnessNRD: Fetched the graph successfully" 
		<< std::endl;
}

template<class GRAPH_t>
double CICPGoodnessNRD_t<GRAPH_t>::getICPEdge(
		const CObservation2DRangeScan& prev_laser_scan,
		const CObservation2DRangeScan& curr_laser_scan,
		constraint_t* rel_edge )
{
	MRPT_START;

	// Uses TParams::ICP member variable
	CSimplePointsMap m1,m2;
	float running_time;
	CICP::TReturnInfo info;

	// TODO - initial estimation?
	pose_t initial_pose;

	m1.insertObservation(&prev_laser_scan);
	m2.insertObservation(&curr_laser_scan);

	CPosePDFPtr pdf = params.icp.Align(
			&m1,
			&m2,
			initial_pose,
			&running_time,
			(void*)&info);

	// return the edge regardless of the goodness of the alignment
	rel_edge->copyFrom(*pdf);

	cout << "rel_edge: " << *rel_edge << endl;
	return info.goodness;

	MRPT_END;
}

template<class GRAPH_t>
void CICPGoodnessNRD_t<GRAPH_t>::convert3DTo2DRangeScan(
		CObservation3DRangeScanPtr& scan3D_in,
		CObservation2DRangeScanPtr* scan2D_out /*= NULL*/) {

	//if ( ((*scan2D_out).null()) ) {
		//// create the 2D laser scan first
		*scan2D_out = CObservation2DRangeScan::Create();
	//}

	if (scan3D_in->hasRangeImage) {
		scan3D_in->convertTo2DScan(**scan2D_out, 
				params.conversion_sensor_label, 
				params.conversion_angle_sup, 
				params.conversion_angle_inf, 
				params.conversion_oversampling_ratio);
	}
	else {
		cout << "Current 3DRangeScan does not contain valid data." << endl;
	}
	cout << "scan2D size: " << (*scan2D_out)->scan.size() << endl;
}


// TParams
//////////////////////////////////////////////////////////////
template<class GRAPH_t>
CICPGoodnessNRD_t<GRAPH_t>::TParams::TParams() { }
template<class GRAPH_t>
CICPGoodnessNRD_t<GRAPH_t>::TParams::~TParams() { }
template<class GRAPH_t>
void CICPGoodnessNRD_t<GRAPH_t>::TParams::dumpToTextStream(
		mrpt::utils::CStream &out) const {
	MRPT_START;

	out.printf("------------------[ ICP Fixed Intervals Node Registration ]------------------\n");
	out.printf("Max distance for registration = %.2f m\n", registration_max_distance);
	out.printf("Max Angle for registration    = %.2f deg\n", RAD2DEG(registration_max_angle));
	out.printf("ICP goodness threshold        = %.2f%% \n", ICP_goodness_thresh*100);
	out.printf("ICP Configuration:\n");
	out.printf("Conversion Sensor label       = %s\n", conversion_sensor_label.c_str());
	out.printf("Conversion angle sup          = %.2f deg\n", RAD2DEG(conversion_angle_sup));
	out.printf("Conversion angle inf          = %.2f deg\n", RAD2DEG(conversion_angle_inf));
	out.printf("Conversion oversampling ratio = %.2f\n", conversion_oversampling_ratio);

	icp.options.dumpToTextStream(out);

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
	ICP_goodness_thresh = source.read_double(
			section,
			"ICP_goodness_thresh",
	 		0.75, false);

	conversion_sensor_label = source.read_string(
			section,
			"conversion_sensor_label",
			"KINECT_TO_2D_SCAN", false);
	conversion_angle_sup = source.read_double(
			section,
			"conversion_angle_sup",
			10, false);
	conversion_angle_sup = DEG2RAD(conversion_angle_sup);
	conversion_angle_inf = source.read_double(
			section,
			"conversion_angle_inf",
			10, false);
	conversion_angle_inf = DEG2RAD(conversion_angle_inf);
	conversion_oversampling_ratio = source.read_double(
			section,
			"conversion_oversampling_ratio",
			1.1, false);


	// load the icp parameters - from "ICP" section explicitly
	icp.options.loadFromConfigFile(source, "ICP");


	std::cout << "Successfully loaded CICPGoodnessNRD parameters. " << std::endl;


	MRPT_END;
}


#endif /* end of include guard: CICPGOODNESSNRD_IMPL_H */
