/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT)               |
	 |                          http://www.mrpt.org/                             |
	 |                                                                           |
	 | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
	 | See: http://www.mrpt.org/Authors - All rights reserved.                   |
	 | Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+ */

// Implementattion file for TSlidingWindow struct
#include "graphslam-precomp.h"  // Precompiled headers

#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/graphslam/misc/TSlidingWindow.h>

using namespace mrpt::graphslam;

TSlidingWindow::TSlidingWindow(
		std::string name /* = "window" */) {
	MRPT_START;

	m_win_size = 5; // just a default value
	m_name = name;

	m_mean_cached = 0;
	m_median_cached = 0;

	m_is_initialized = false;
	m_mean_updated = false;
	m_median_updated = false;
	m_std_dev_updated = false;

	MRPT_END;
}
TSlidingWindow::~TSlidingWindow() { }

double TSlidingWindow::getMedian() {
	MRPT_START;

	double median_out = 0.0;
	if (m_measurements_vec.empty()) {
		return 0.0;
	}

	if (m_median_updated) {
		median_out = m_median_cached;
	}
	else {
		// copy the current vector, sort it and return value in middle
		std::vector<double> vec_sorted(m_measurements_vec);
		std::sort(vec_sorted.begin(), vec_sorted.end());

		median_out = vec_sorted.at(vec_sorted.size()/2);

		m_median_cached = median_out;
		m_median_updated = true;
	}

	return median_out;

	MRPT_END;
}
double TSlidingWindow::getMean() {
	MRPT_START;

	double mean_out = 0.0;

	if (m_mean_updated) {
		mean_out = m_mean_cached;
	}
	else {
		mean_out = std::accumulate(m_measurements_vec.begin(), m_measurements_vec.end(), 0.0);
		mean_out /= m_measurements_vec.size();

		m_mean_cached = mean_out;
		m_mean_updated = true;
	}

	return mean_out;

	MRPT_END;
}
double TSlidingWindow::getStdDev() {
	MRPT_START;

	double std_dev_out = 0.0;

	if (m_std_dev_updated) { // return the cached version?
		std_dev_out = m_std_dev_cached;
	}
	else {
		double mean = this->getMean();

		double sum_of_sq_diffs = 0;
		for (std::vector<double>::const_iterator it = m_measurements_vec.begin();
				it != m_measurements_vec.end(); ++it) {
			sum_of_sq_diffs += std::pow(*it - mean, 2);
		}
		std_dev_out = sqrt(sum_of_sq_diffs / m_win_size);

		m_std_dev_cached = std_dev_out;
		m_std_dev_updated = true;
	}

	return std_dev_out;
	MRPT_END;
}

bool TSlidingWindow::evaluateMeasurementInGaussian(double measurement) {
	// get the boundaries for acceptance of measurements - [-3sigma, 3sigma] with
	// regards to the mean
	double low_lim = this->getMean() - 3*this->getStdDev();
	double upper_lim = this->getMean() + 3*this->getStdDev();

	return measurement > low_lim && measurement < upper_lim;
}
bool TSlidingWindow::evaluateMeasurementAbove(
		double value) {
	MRPT_START;

	double threshold = this->getMean();
	return (value > threshold);

	MRPT_END;
}
bool TSlidingWindow::evaluateMeasurementBelow(
		double value) {
	return !evaluateMeasurementAbove(value);
}


void TSlidingWindow::addNewMeasurement(
		double measurement ) {
	MRPT_START;

	m_is_initialized = true;

	// if I haven't already filled up to win_size the vector, just add it
	if ( m_win_size > m_measurements_vec.size() ) {
		m_measurements_vec.push_back(measurement);
	}
	else {
		// remove first element - add it as last element
		m_measurements_vec.erase(m_measurements_vec.begin());
		m_measurements_vec.push_back(measurement);
	}

	m_mean_updated = false;
	m_median_updated = false;
	m_std_dev_updated = false;

	MRPT_END;
}
void TSlidingWindow::resizeWindow(
		size_t new_size ) {
	MRPT_START;

	size_t curr_size = m_measurements_vec.size();
	if ( new_size < curr_size ) {
		// remove (curr_size - new_size) elements from the beginning of the
		// measurements vector
		m_measurements_vec.erase(m_measurements_vec.begin(),
				m_measurements_vec.begin() + (curr_size - new_size));

		m_mean_updated = false;
		m_median_updated = false;
	}

	m_win_size = new_size;

	MRPT_END;
}
void TSlidingWindow::loadFromConfigFile(
		const mrpt::utils::CConfigFileBase& source,
		const std::string& section) {
	MRPT_START;

	size_t sliding_win_size = source.read_int(
			section,
			"sliding_win_size",
			10, false);
	this->resizeWindow(sliding_win_size);

	MRPT_END;
}
void TSlidingWindow::dumpToTextStream(
		mrpt::utils::CStream &out) const {
	MRPT_START;

	out.printf("-----------[ %s: Sliding Window Properties ]-----------\n",
			m_name.c_str());
	out.printf("Measurements Vector: \n");
	for (std::vector<double>::const_iterator it = m_measurements_vec.begin();
			it != m_measurements_vec.end(); ++it ) {
		out.printf("\t%.2f\n", *it);
	}
	out.printf("\n");

	out.printf("m_name              : %s\n"   , m_name.c_str());
	out.printf("m_mean_cached       : %.2f\n" , m_mean_cached);
	out.printf("m_median_cached     : %.2f\n" , m_median_cached);
	out.printf("m_std_dev_cached    : %.2f\n" , m_std_dev_cached);
	out.printf("m_mean_updated      : %s\n"   , m_mean_updated? "TRUE": "FALSE");
	out.printf("m_median_updated    : %s\n"   , m_median_updated? "TRUE": "FALSE");
	out.printf("m_std_dev_updated   : %s\n"   , m_std_dev_updated? "TRUE": "FALSE");
	out.printf("m_win_size          : %lu\n"  , m_win_size);
	out.printf("m_is_initialized    : %s\n"   , m_is_initialized? "TRUE": "FALSE");

	MRPT_END;
}

size_t TSlidingWindow::getWindowSize() const {
	return m_win_size;
}

bool TSlidingWindow::windowIsFull() const {
	return (m_win_size == m_measurements_vec.size());

}
