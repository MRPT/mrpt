/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "graphslam-precomp.h"  // Precompiled headers
#include <mrpt/graphslam/misc/CEdgeCounter.h>

// implementation file of CEdgeCounter class
using namespace mrpt::graphslam::supplementary;


CEdgeCounter::CEdgeCounter() {
	this->initCEdgeCounter();

}

CEdgeCounter::~CEdgeCounter() {}

void CEdgeCounter::initCEdgeCounter() {

	this->clearAllEdges();
	m_has_read_textmessage_params = false;

	// visualization parameters for total edges / loop closures
	m_display_total_edges = false;
	m_display_loop_closures = false;

	m_offset_y_total_edges = 0.0;
	m_offset_y_loop_closures = 0.0;

	m_text_index_total_edges = 0;
	m_text_index_loop_closures = 0;

	m_unique_edges = 0;

	m_win = NULL;
	m_win_manager = NULL;

}

void CEdgeCounter::setWindowManagerPtr(
		mrpt::graphslam::CWindowManager* win_manager) {
	MRPT_START;
	ASSERT_(win_manager);
	ASSERT_(win_manager->win);

	m_win_manager = win_manager;
	m_win = m_win_manager->win;

	MRPT_END;
}

void CEdgeCounter::setRemovedEdges(int removed_edges) {
	m_unique_edges = this->getTotalNumOfEdges() - removed_edges;
}

void CEdgeCounter::setLoopClosureEdgesManually(int num_loop_closures) {
	m_num_loop_closures = num_loop_closures;
}

int CEdgeCounter::getLoopClosureEdges() const {
	return m_num_loop_closures;
}

int CEdgeCounter::getTotalNumOfEdges() const {
	int sum = 0;

	for (std::map<std::string, int>::const_iterator it =
			m_name_to_edges_num.begin(); it != m_name_to_edges_num.end(); ++it) {
		sum += it->second;
	}
	return sum;
}

void CEdgeCounter::getTotalNumOfEdges(int* total_num_edges) const {
	int sum = 0;

	for (std::map<std::string, int>::const_iterator it =
			m_name_to_edges_num.begin(); it != m_name_to_edges_num.end(); ++it) {
		sum += it->second;
	}
	*total_num_edges = sum;
}

int CEdgeCounter::getNumForEdgeType(const std::string& name) const {
	std::map<std::string, int>::const_iterator search =
		m_name_to_edges_num.find(name);
	if ( search != m_name_to_edges_num.end() ) {
		return search->second;
	}
	else {
		THROW_EXCEPTION("No edge with such name exists")
	}
}

void CEdgeCounter::getNumForEdgeType(const std::string& name, int* total_num) {

	std::map<std::string, int>::const_iterator search =
		m_name_to_edges_num.find(name);
	if ( search != m_name_to_edges_num.end() ) {
		*total_num = search->second;
	}
	else {
		THROW_EXCEPTION("No edge with such name exists")
	}
}

void CEdgeCounter::setEdgesManually(const std::string& name, int num_of_edges) {
	std::map<std::string, int>::iterator search = m_name_to_edges_num.find(name);
	if ( search != m_name_to_edges_num.end() ) {
		search->second = num_of_edges;
	}
	else {
		std::string str_err = "No edge with such name exists.";
		THROW_EXCEPTION(str_err)
	}
	// Update the visualization if the user has already set the vizualization
	// parameters
	if (m_has_read_textmessage_params && m_win_manager) {
		this->updateTextMessages();
	}
}

void CEdgeCounter::addEdge(const std::string& name,
		bool is_loop_closure /* =false */,
		bool is_new /* =false */) {
	std::map<std::string, int>::iterator search = m_name_to_edges_num.find(name);
	if ( search != m_name_to_edges_num.end() ) {
		(search->second)++; // increment to the found element

		// specify warning if is_new = true
		if (is_new) {
			std::string str_err = "Specified edge type already exists but is_new is also specified!";
			THROW_EXCEPTION(str_err)
				//std::stringstream ss_warn;
				//ss_warn << "Commencing with the increment normally" << std::endl;
		}
		if (is_loop_closure) {
			// throw if user has also specified the is new boolean flag
			if (is_new) {
				std::string str_err = "Both is_new and is_loop_closure flags are true. Exiting...";
				THROW_EXCEPTION(str_err)
			}
			m_num_loop_closures++;
		}
	}
	else {
		if (is_new) {
			m_name_to_edges_num[name] = 1;
		}
		else {
			std::string str_err = "No edge with such name exists. Specify is_new parameter if you want to add it";
			THROW_EXCEPTION(str_err)
		}
	}

	// Update the visualization if the user has already set the vizualization
	// parameters
	if (m_has_read_textmessage_params && m_win_manager) {
		updateTextMessages();
	}
}

void CEdgeCounter::addEdgeType(const std::string& name) {
	std::map<std::string, int>::const_iterator search =
		m_name_to_edges_num.find(name);

	if ( search != m_name_to_edges_num.end() ) {
		THROW_EXCEPTION(
				mrpt::format("Specified edge type %s already exists", name.c_str()))
	}
	else {
		m_name_to_edges_num[name] = 0;
	}
}

void CEdgeCounter::clearAllEdges() {
	m_num_loop_closures = 0;

	m_name_to_edges_num.clear();
	m_name_to_offset_y.clear();
	m_name_to_text_index.clear();

	m_has_read_textmessage_params = false;
	m_display_total_edges					= false;
	m_display_loop_closures				= false;
}

void CEdgeCounter::dumpToConsole() const {
	std::string str(getAsString());
	std::cout << str << std::endl;
}

void CEdgeCounter::getAsString(std::string* str_out) const {
	std::stringstream ss_out;
	std::string sep(80, '#');
	ss_out << "Summary of Edges: " << std::endl;
	ss_out << sep << std::endl;

	ss_out << "\tTotal registered edges: "
		<< this->getTotalNumOfEdges() << std::endl;
	ss_out << "\tUnique edges (after removal of multiple edges connecting the same nodes): "
		<< m_unique_edges << std::endl;

	for (std::map<std::string, int>::const_iterator it =
			m_name_to_edges_num.begin(); it != m_name_to_edges_num.end(); ++it) {
		ss_out << "\t" << it->first << " edges: " << it->second << std::endl;
	}
	ss_out << "\tLoop closure edges: " << this->getLoopClosureEdges() << std::endl;

	// dump the contents to the provided string
	*str_out = ss_out.str();
}

std::string CEdgeCounter::getAsString() const {
	std::string str;
	this->getAsString(&str);
	return str;
}

// VISUALIZATION RELATED METHODS
// ////////////////////////////

void CEdgeCounter::setTextMessageParams(
		const std::map<std::string, double>& name_to_offset_y,
		const std::map<std::string, int>& name_to_text_index) {
	ASSERTMSG_(m_win_manager,
			"Visualization of data was requested but no CWindowManager pointer was provided");
	ASSERT_EQUAL_(name_to_offset_y.size(), name_to_text_index.size());

	for (std::map<std::string, double>::const_iterator it =
			name_to_offset_y.begin(); it != name_to_offset_y.end(); ++it) {
		std::string name = it->first;

		// check if name already exist, otherwise throw exception
		std::map<std::string, int>::const_iterator search =
			m_name_to_edges_num.find(name);
		if ( search == m_name_to_edges_num.end() ) {
			std::stringstream ss_err;
			ss_err << "Name " << name << " is not recognized as an Edge type."
				<< std::endl;
			THROW_EXCEPTION(ss_err.str())
		}
		// name exists ...

		double offset_y = it->second;
		int text_index = name_to_text_index.find(name)->second;

		m_name_to_offset_y[name] = offset_y;
		m_name_to_text_index[name] = text_index;

	}

	m_has_read_textmessage_params = true;
}

void CEdgeCounter::setTextMessageParams(
		const std::map<std::string, double>& name_to_offset_y,
		const std::map<std::string, int>& name_to_text_index,
		const double& offset_y_total_edges, const int& text_index_total_edges,
		const double& offset_y_loop_closures, const int& text_index_loop_closures) {

	// set the parameters for total edges / loop closures
	m_display_total_edges = true;
	m_display_loop_closures = true;

	m_offset_y_total_edges = offset_y_total_edges;
	m_offset_y_loop_closures = offset_y_loop_closures;

	m_text_index_total_edges = text_index_total_edges;
	m_text_index_loop_closures = text_index_loop_closures;

	// pass execution to the other setTextMessageParams
	this->setTextMessageParams(name_to_offset_y, name_to_text_index);

}

void CEdgeCounter::updateTextMessages() const {
	ASSERT_(m_win_manager);
	ASSERT_(m_has_read_textmessage_params);
	ASSERT_EQUAL_(m_name_to_offset_y.size(), m_name_to_text_index.size());

	//Add text message for the total amount of edges
	std::stringstream title;
	title << "Total edges: " <<  this->getTotalNumOfEdges();
	//if (m_unique_edges) {
	//title << " |Unique: " << m_unique_edges << std::endl;
	//}
	if (m_display_total_edges && m_win_manager) {
		m_win_manager->addTextMessage(5,-m_offset_y_total_edges,
				title.str(),
				mrpt::utils::TColorf(1.0, 1.0, 1.0),
				/* unique_index = */ m_text_index_total_edges);
	}

	// add a textMessage for every stored edge type
	for (std::map<std::string, double>::const_iterator it = m_name_to_offset_y.begin();
			it != m_name_to_offset_y.end(); ++it) {

		std::string name = it->first;
		double offset_y = it->second;
		int text_index = m_name_to_text_index.find(name)->second;
		int edges_num = m_name_to_edges_num.find(name)->second;

		std::stringstream title;
		title << "  " << name << ": " <<	edges_num << std::endl;
		m_win_manager->addTextMessage(5,-offset_y,
				title.str(),
				mrpt::utils::TColorf(1.0, 1.0, 1.0),
				/* unique_index = */ text_index);
	}


	// add text message for the loop closures
	if (m_display_loop_closures) {
		std::stringstream title;
		title << "  " << "Loop closures: " <<  m_num_loop_closures << std::endl;
		m_win_manager->addTextMessage(5,-m_offset_y_loop_closures,
				title.str(),
				mrpt::utils::TColorf(1.0, 1.0, 1.0),
				/* unique_index = */ m_text_index_loop_closures);
	}

	m_win->forceRepaint();
}
