/* +---------------------------------------------------------------------------+
	 |                     Mobile Robot Programming Toolkit (MRPT)               |
	 |                          http://www.mrpt.org/                             |
	 |                                                                           |
	 | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
	 | See: http://www.mrpt.org/Authors - All rights reserved.                   |
	 | Released under BSD License. See details in http://www.mrpt.org/License    |
	 +---------------------------------------------------------------------------+ */

#ifndef CREGISTRATIONDECIDEROROPTIMIZER_IMPL_H
#define CREGISTRATIONDECIDEROROPTIMIZER_IMPL_H

using namespace mrpt::graphslam;

template<class GRAPH_t>
CRegistrationDeciderOrOptimizer<GRAPH_t>::CRegistrationDeciderOrOptimizer():
	m_graph(NULL),
	m_graph_section(NULL),
	m_win_manager(NULL),
	m_win(NULL),
	m_win_observer(NULL),
	m_initialized_visuals(false),
	m_class_name("CRegistrationDeciderOrOptimizer") { }


template<class GRAPH_t>
CRegistrationDeciderOrOptimizer<GRAPH_t>::~CRegistrationDeciderOrOptimizer() {

}

template<class GRAPH_t>
void CRegistrationDeciderOrOptimizer<GRAPH_t>::initializeLoggers(
		std::string class_name) {
	using namespace std;
	using namespace mrpt::utils;

	this->m_class_name = class_name;

	this->m_time_logger.setName(this->m_class_name);
	this->logging_enable_keep_record = true;
	this->setLoggerName(this->m_class_name);

	// just for the first message, set it to debug.
	this->setMinLoggingLevel(LVL_DEBUG);
	MRPT_LOG_DEBUG_STREAM << "Initialized time, output logger instances." << endl;
}


template<class GRAPH_t>
void CRegistrationDeciderOrOptimizer<GRAPH_t>::setWindowManagerPtr(
		mrpt::graphslam::CWindowManager* win_manager) {
	ASSERT_(win_manager);
	m_win_manager = win_manager;

	if (m_win_manager) {
		m_win = m_win_manager->win;

		m_win_observer = m_win_manager->observer;
	}
}

template<class GRAPH_t>
void CRegistrationDeciderOrOptimizer<GRAPH_t>::setCriticalSectionPtr(
		mrpt::synch::CCriticalSection* graph_section) {

	m_graph_section = graph_section;
	this->logFmt(mrpt::utils::LVL_DEBUG, "Fetched the CCRiticalSection successfully");
}

template<class GRAPH_t>
void CRegistrationDeciderOrOptimizer<GRAPH_t>::initializeVisuals() {
	using namespace mrpt::utils;
	MRPT_LOG_DEBUG_STREAM << "Initializing visuals";

	this->assertVisualsVars();
	m_initialized_visuals = true;
}

template<class GRAPH_t>
void CRegistrationDeciderOrOptimizer<GRAPH_t>::assertVisualsVars() {
	ASSERTMSG_(this->m_win, "No CDisplayWindow3D* was provided");
	ASSERTMSG_(this->m_win_manager, "No CWindowManager* was provided");
	ASSERTMSG_(this->m_win_observer, "No CWindowObserver* was provided");
}

template<class GRAPH_t>
void CRegistrationDeciderOrOptimizer<GRAPH_t>::updateVisuals() {
	ASSERT_(m_initialized_visuals);
	MRPT_LOG_DEBUG_STREAM << "Updating visuals";
}

template<class GRAPH_t>
void CRegistrationDeciderOrOptimizer<GRAPH_t>::notifyOfWindowEvents(
		const std::map<std::string, bool>& events_occurred) {
	this->assertVisualsVars();
	MRPT_LOG_DEBUG_STREAM << "Querrying window events...";
}

template<class GRAPH_t>
void CRegistrationDeciderOrOptimizer<GRAPH_t>::loadParams(
		const std::string& source_fname) {
	MRPT_LOG_DEBUG_STREAM << "Loading corresponding parameters";
}

template<class GRAPH_t>
void CRegistrationDeciderOrOptimizer<GRAPH_t>::printParams() const {
	MRPT_LOG_DEBUG_STREAM << "Printing corresponding parameters";
}

template<class GRAPH_t>
void CRegistrationDeciderOrOptimizer<GRAPH_t>::getDescriptiveReport(
		std::string* report_str) const {
	MRPT_LOG_DEBUG_STREAM << "Generating corresponding report";
	// TODO - give the compact form here!

}

template<class GRAPH_t>
void CRegistrationDeciderOrOptimizer<GRAPH_t>::setGraphPtr(GRAPH_t* graph) {
	using namespace mrpt::utils;

	m_graph = graph;
	MRPT_LOG_DEBUG_STREAM << "Fetched the graph pointer successfully";
}

#endif /* end of include guard: CREGISTRATIONDECIDEROROPTIMIZER_IMPL_H */
