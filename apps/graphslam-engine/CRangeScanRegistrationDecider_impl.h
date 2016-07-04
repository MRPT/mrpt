/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CRANGESCANREGISTRATIONDECIDER_IMPL_H
#define CRANGESCANREGISTRATIONDECIDER_IMPL_H

using namespace mrpt::graphslam::deciders;

template<class GRAPH_t>
void CRangeScanRegistrationDecider_t<GRAPH_t>::getICPEdge(
		const mrpt::obs::CObservation2DRangeScan& from,
		const mrpt::obs::CObservation2DRangeScan& to,
		constraint_t* rel_edge,
		mrpt::poses::CPose2D* initial_pose_in/* = NULL */,
		mrpt::slam::CICP::TReturnInfo* icp_info/* = NULL */) {
	MRPT_START;

	mrpt::maps::CSimplePointsMap m1,m2;
	float running_time;
	mrpt::slam::CICP::TReturnInfo info;

	// have them initialized prior - and then just clear them
	m1.insertObservation(&from);
	m2.insertObservation(&to);

	// If given, use initial_pose_in as a first guess for the ICP
	mrpt::poses::CPose2D initial_pose;
	if (initial_pose_in) {
		initial_pose = *initial_pose_in;
	}

	mrpt::poses::CPosePDFPtr pdf = params.icp.Align(
			&m1,
			&m2,
			initial_pose,
			&running_time,
			(void*)&info);

	//std::cout << "ICP Alignment operation: \n" 
		//<< "\n\t nIterations: " << info.nIterations
		//<< "\n\t quality: " << info.quality
		//<< "\n\t goodness: " << info.goodness
		//<< std::endl;
	//std::cout << "Alignemnt took: " << running_time << " s" << std::endl;


	// return the edge regardless of the goodness of the alignment
	rel_edge->copyFrom(*pdf);

	// if given, fill the TReturnInfo Struct
	if (icp_info) {
		icp_info->nIterations = info.nIterations;
		icp_info->goodness = info.goodness;
		icp_info->quality = info.quality;
		icp_info->cbSize = info.cbSize;
	}

	MRPT_END;
}
template<class GRAPH_t>
void CRangeScanRegistrationDecider_t<GRAPH_t>::getICPEdge(
		mrpt::obs::CObservation3DRangeScan& from,
		mrpt::obs::CObservation3DRangeScan& to,
		constraint_t* rel_edge,
		mrpt::poses::CPose2D* initial_pose_in /* =NULL */,
		mrpt::slam::CICP::TReturnInfo* icp_info /* =NULL */) {
	MRPT_START;

	ASSERTMSG_(from.hasRangeImage, 
			mrpt::format("Laser scan doesn't contain valid range image"));
	ASSERTMSG_(to.hasRangeImage, 
			mrpt::format("Laser scan doesn't contain valid range image"));

	// TODO - have this as a class member
	mrpt::maps::CSimplePointsMap m1,m2;
	float running_time;
	mrpt::slam::CICP::TReturnInfo info;

	m1.insertObservation(&from);
	m2.insertObservation(&to);

	this->decimatePointsMap(&m1, /* keep every = */ 40, /* low_lim = */ 5000);
	this->decimatePointsMap(&m2, /* keep every = */ 40, /* low_lim = */ 5000);

	// If given, use initial_pose_in as a first guess for the ICP
	mrpt::poses::CPose3D initial_pose;
	if (initial_pose_in) {
		initial_pose = *initial_pose_in;
	}

	mrpt::poses::CPose3DPDFPtr pdf = params.icp.Align3D(
			&m1,
			&m2,
			initial_pose,
			&running_time,
			(void*)&info);

	//CStdOutStream cout_stream;
	//std::cout << "For first laser scan: " << std::endl;
	//from.getDescriptionAsText(std::cout);
	//std::cout << "For second laser scan: " << std::endl;
	//to.getDescriptionAsText(std::cout);

	std::cout << "ICP Alignment operation: \n" 
		<< "\t nIterations: " << info.nIterations
		//<< "\t quality: " << info.quality
		<< "\t goodness: " << info.goodness
		<< std::endl;
	std::cout << "Alignemnt took: " << running_time << " s" << std::endl;


	// return the edge regardless of the goodness of the alignment
	// copy fro the 3D PDF
	rel_edge->copyFrom(*pdf);

	//std::cout << "3D Pose from alignment: " << pdf->getMeanVal() << std::endl;
	//std::cout << "2D corresponding pose : " << rel_edge->getMeanVal() << std::endl;

	// if given, fill the TReturnInfo Struct
	if (icp_info) {
		icp_info->nIterations = info.nIterations;
		icp_info->goodness = info.goodness;
		icp_info->quality = info.quality;
		icp_info->cbSize = info.cbSize;
	}


	MRPT_END;
}

template<class GRAPH_t>
void CRangeScanRegistrationDecider_t<GRAPH_t>::decimatePointsMap(
		mrpt::maps::CPointsMap* m,
		size_t keep_point_every, /* = 4 */
		size_t low_lim /* = 8000 */) {
	MRPT_START;

	size_t map_size = m->size();

	if (low_lim) {
		// check if current keep_point_every variable is too large
		size_t conservative_keep_point_every = map_size / low_lim;
		keep_point_every = std::min(keep_point_every, conservative_keep_point_every);
	}

	// insert a false every "keep_point_every" points
	std::vector<bool> deletion_mask(map_size, true);
	for (size_t i = 0; i != map_size; ++i) {
		if (i % keep_point_every == 0) {
			deletion_mask[i] = false;
		}
	}
	m->applyDeletionMask(deletion_mask);
	
	std::cout << "Map size: " << map_size << " => " << m->size() << std::endl;

	MRPT_END;
}

template<class GRAPH_t>
void CRangeScanRegistrationDecider_t<GRAPH_t>::convert3DTo2DRangeScan(
		mrpt::obs::CObservation3DRangeScanPtr& scan3D_in,
		mrpt::obs::CObservation2DRangeScanPtr* scan2D_out /*= NULL*/) {

	// create the 2D laser scan first
	if ( (*scan2D_out).null() ) {
		*scan2D_out = mrpt::obs::CObservation2DRangeScan::Create();
	}

	if (scan3D_in->hasRangeImage) {
		scan3D_in->convertTo2DScan(**scan2D_out, 
				params.conversion_sensor_label, 
				params.conversion_angle_sup, 
				params.conversion_angle_inf, 
				params.conversion_oversampling_ratio);
	}
	else {
		std::cout << "No valid rangeImage found" << std::endl;
	}
}

// TSlidingWindow
// //////////////////////////////////

template<class GRAPH_t>
CRangeScanRegistrationDecider_t<GRAPH_t>::TSlidingWindow::TSlidingWindow(
		double win_size /* = 10 */ ) {
	MRPT_START;

	m_win_size = win_size;

	// use mean by default for deciding whether to accept an ICP goodenss value (
	// goodness > getMean() )
	m_evaluate_using_mean = true;

	m_is_initialized = false;
	m_mean_updated = false;
	m_median_updated = false;
	
	MRPT_END;
}
template<class GRAPH_t>
CRangeScanRegistrationDecider_t<GRAPH_t>::TSlidingWindow::~TSlidingWindow() { }
template<class GRAPH_t>
inline double CRangeScanRegistrationDecider_t<GRAPH_t>::TSlidingWindow::getMedian() {
	MRPT_START;

	double median_out = 0.0;
	if (m_goodness_vec.empty()) {
		return 0.0;
	}
	if (m_median_updated) {
		median_out = m_median_cached;
	}
	else {
		// copy the current goodness vector, sort it and return value in middle
		std::vector<double> goodness_vec_sorted(m_goodness_vec);
		std::sort(goodness_vec_sorted.begin(), goodness_vec_sorted.end());

		//for (std::vector<double>::const_iterator it = m_goodness_vec.begin();
				//it != m_goodness_vec.end(); ++it ) {
			//std::cout << "vector sorted: " << std::endl;
			//ss_out << "\t" << *it << std::endl;
		//}

		median_out = goodness_vec_sorted.at(goodness_vec_sorted.size()/2);

		m_median_cached = median_out;
		m_median_updated = true;
	}

	return median_out;

	MRPT_END;
}
template<class GRAPH_t>
inline double CRangeScanRegistrationDecider_t<GRAPH_t>::TSlidingWindow::getMean() {
	MRPT_START;

	double m_mean_out = 0.0;

	if (m_mean_updated) {
		m_mean_out = m_mean_cached;
	}
	else {
		m_mean_out = std::accumulate(m_goodness_vec.begin(), m_goodness_vec.end(), 0.0);
		m_mean_out /= m_goodness_vec.size();

		m_mean_cached = m_mean_out;
		m_mean_updated = true;
	}

	return m_mean_out;

	MRPT_END;
}
template<class GRAPH_t>
inline bool CRangeScanRegistrationDecider_t<GRAPH_t>::TSlidingWindow::evaluateICPgoodness( 
		double goodness) {
	MRPT_START;

	double threshold = 0;

	if (m_evaluate_using_mean) {
		threshold = this->getMean();
	}
	else  {
		threshold = this->getMedian();
	}

	return (goodness > threshold);

	MRPT_END;
}

template<class GRAPH_t>
inline void CRangeScanRegistrationDecider_t<GRAPH_t>::TSlidingWindow::addNewMeasurement(
		double goodness_val ) {
	MRPT_START;

	m_is_initialized = true;

	// if I haven't already filled up to win_size the vector, just add it
	if ( m_win_size > m_goodness_vec.size() ) {
		m_goodness_vec.push_back(goodness_val);
	}
	else {
		// remove first element - add it as last element
		m_goodness_vec.erase(m_goodness_vec.begin());
		m_goodness_vec.push_back(goodness_val);
	}

	m_mean_updated = false;
	m_median_updated = false;

	MRPT_END;
}
template<class GRAPH_t>
void CRangeScanRegistrationDecider_t<GRAPH_t>::TSlidingWindow::resizeWindow(
		size_t new_size ) {
	MRPT_START;

	size_t curr_size = m_goodness_vec.size();
	if ( new_size < curr_size ) {
		// remove (curr_size - new_size) elements from the beginning of the
		// goodness vector
		m_goodness_vec.erase(m_goodness_vec.begin(), 
				m_goodness_vec.begin() + (curr_size - new_size));

		m_mean_updated = false;
		m_median_updated = false;
	}

	m_win_size = new_size;

	MRPT_END;
}
template<class GRAPH_t>
void CRangeScanRegistrationDecider_t<GRAPH_t>::TSlidingWindow::loadFromConfigFile(
		const mrpt::utils::CConfigFileBase& source,
    const std::string& section) {
  MRPT_START;
	
	size_t sliding_win_size = source.read_int(
			section,
			"sliding_win_size",
			10, false);
	std::string compute_ICP_threshold_using  = source.read_string(
			section,
			"compute_ICP_threshold_using",
			"mean", false);

	this->resizeWindow(sliding_win_size);
	this->setEvaluationCriterion(compute_ICP_threshold_using);

	std::cout << "section: " << section << std::endl;
	std::cout << "sliding_win_size: " << sliding_win_size << std::endl;
	std::cout << "m_win_size: " << m_win_size << std::endl;

  MRPT_END;
}
template<class GRAPH_t>
void CRangeScanRegistrationDecider_t<GRAPH_t>::TSlidingWindow::dumpToTextStream(
		mrpt::utils::CStream &out) const {
	MRPT_START;

	out.printf("-----------[ Sliding Window Properties ]-----------\n");
	out.printf("Goodness Vector: \n");
	for (std::vector<double>::const_iterator it = m_goodness_vec.begin();
			it != m_goodness_vec.end(); ++it ) {
		out.printf("\t%.2f\n", *it);
	}
	out.printf("\n");

	out.printf("m_mean_cached       : %.2f\n" , m_mean_cached);
	out.printf("m_median_cached     : %.2f\n" , m_median_cached);
	out.printf("m_mean_updated      : %d\n"   , m_mean_updated);
	out.printf("m_median_updated    : %d\n"   , m_median_updated);
	out.printf("m_win_size          : %lu\n"  , m_win_size);
	out.printf("m_is_initialized    : %d\n"   , m_is_initialized);
	out.printf("Evaluate using mean : %d\n"   , m_evaluate_using_mean);

	MRPT_END;
}
template<class GRAPH_t>
void CRangeScanRegistrationDecider_t<GRAPH_t>::TSlidingWindow::setEvaluationCriterion(std::string criterion) {
	MRPT_START;

	ASSERTMSG_(mrpt::system::strCmpI(criterion, "mean") ||
			mrpt::system::strCmpI(criterion, "median"),
			format("\nEvaluation criterion %s is not available\n", 
				criterion.c_str()));

	m_evaluate_using_mean = 
		mrpt::system::strCmpI(criterion, "mean") ? true : false;


	MRPT_END;
}


// TParameter
// //////////////////////////////////

template<class GRAPH_t>
CRangeScanRegistrationDecider_t<GRAPH_t>::TParams::TParams():
	has_read_config(false)
{ }

template<class GRAPH_t>
CRangeScanRegistrationDecider_t<GRAPH_t>::TParams::~TParams() {
}

template<class GRAPH_t>
void CRangeScanRegistrationDecider_t<GRAPH_t>::TParams::dumpToTextStream(
		mrpt::utils::CStream &out) const {
	MRPT_START;
	out.printf("3D=>2D LaserScan Conversion Sensor label       = %s\n",
			conversion_sensor_label.c_str());
	out.printf("3D=>2D LaserScan Conversion angle sup          = %.2f deg\n",
			mrpt::utils::RAD2DEG(conversion_angle_sup));
	out.printf("3D=>2D LaserScan Conversion angle inf          = %.2f deg\n",
			mrpt::utils::RAD2DEG(conversion_angle_inf));
	out.printf("3D=>2D LaserScan Conversion oversampling ratio = %.2f\n",
			conversion_oversampling_ratio);

	icp.options.dumpToTextStream(out);

	MRPT_END;
}
template<class GRAPH_t>
void CRangeScanRegistrationDecider_t<GRAPH_t>::TParams::loadFromConfigFile(
		const mrpt::utils::CConfigFileBase& source,
    const std::string& section) {
  MRPT_START;

	conversion_sensor_label = source.read_string(
			section,
			"conversion_sensor_label",
			"KINECT_TO_2D_SCAN", false);
	conversion_angle_sup = source.read_double(
			section,
			"conversion_angle_sup",
			10, false);
	conversion_angle_sup = mrpt::utils::DEG2RAD(conversion_angle_sup);
	conversion_angle_inf = source.read_double(
			section,
			"conversion_angle_inf",
			10, false);
	conversion_angle_inf = mrpt::utils::DEG2RAD(conversion_angle_inf);
	conversion_oversampling_ratio = source.read_double(
			section,
			"conversion_oversampling_ratio",
			1.1, false);

	// load the icp parameters - from "ICP" section explicitly
	icp.options.loadFromConfigFile(source, "ICP");

	has_read_config = true;

	MRPT_END;
}




#endif /* end of include guard: CRANGESCANREGISTRATIONDECIDER_IMPL_H */
