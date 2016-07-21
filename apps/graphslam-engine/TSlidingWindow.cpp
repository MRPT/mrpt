/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT)               |
	 |                          http://www.mrpt.org/                             |
	 |                                                                           |
	 | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
	 | See: http://www.mrpt.org/Authors - All rights reserved.                   |
	 | Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+ */

// Implementattion file for TSlidingWindow struct

#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/utils/CStream.h>

#include "TSlidingWindow.h"

TSlidingWindow::TSlidingWindow(
		std::string name /* = "window" */) {
	MRPT_START;

	m_win_size = 5; // just a default value
	m_name = name;

	m_is_initialized = false;
	m_mean_updated = false;
	m_median_updated = false;

	MRPT_END;
}
TSlidingWindow::~TSlidingWindow() { }

double TSlidingWindow::getMedian() {
	MRPT_START;

	double median_out = 0.0;
	if (m_goodness_vec.empty()) {
		return 0.0;
	}

	if (m_median_updated) {
		median_out = m_median_cached;
	}
	else {
		// copy the current vector, sort it and return value in middle
		std::vector<double> goodness_vec_sorted(m_goodness_vec);
		std::sort(goodness_vec_sorted.begin(), goodness_vec_sorted.end());

		median_out = goodness_vec_sorted.at(goodness_vec_sorted.size()/2);

		m_median_cached = median_out;
		m_median_updated = true;
	}

	return median_out;

	MRPT_END;
}
double TSlidingWindow::getMean() {
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
// TODO - make it use the boundaries
bool TSlidingWindow::evaluateMeasurementAbove(
		double value) {
	MRPT_START;

	double threshold = this->getMean();
	return (value > threshold);

	MRPT_END;
}

void TSlidingWindow::addNewMeasurement(
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
void TSlidingWindow::resizeWindow(
		size_t new_size ) {
	MRPT_START;

	size_t curr_size = m_goodness_vec.size();
	if ( new_size < curr_size ) {
		// remove (curr_size - new_size) elements from the beginning of the
		// measurements vector
		m_goodness_vec.erase(m_goodness_vec.begin(),
				m_goodness_vec.begin() + (curr_size - new_size));

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

	MRPT_END;
}
