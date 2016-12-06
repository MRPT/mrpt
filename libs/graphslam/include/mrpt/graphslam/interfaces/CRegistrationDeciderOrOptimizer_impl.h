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
	m_win_manager(NULL),
	m_win(NULL),
	m_win_observer(NULL),
	m_graph(NULL) { }


template<class GRAPH_t>
CRegistrationDeciderOrOptimizer<GRAPH_t>::~CRegistrationDeciderOrOptimizer() {

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
}

template<class GRAPH_t>
void CRegistrationDeciderOrOptimizer<GRAPH_t>::initializeVisuals() {
	ASSERT_(m_win_manager);
}

template<class GRAPH_t>
void CRegistrationDeciderOrOptimizer<GRAPH_t>::updateVisuals() {
	ASSERT_(m_win_manager);

}

template<class GRAPH_t>
void CRegistrationDeciderOrOptimizer<GRAPH_t>::notifyOfWindowEvents(
		const std::map<std::string, bool>& events_occurred) {
	ASSERT_(m_win_manager);
}

template<class GRAPH_t>
void CRegistrationDeciderOrOptimizer<GRAPH_t>::loadParams(
		const std::string& source_fname) {
}

template<class GRAPH_t>
void CRegistrationDeciderOrOptimizer<GRAPH_t>::printParams() const {
}

template<class GRAPH_t>
void CRegistrationDeciderOrOptimizer<GRAPH_t>::getDescriptiveReport(
		std::string* report_str) const {

}

template<class GRAPH_t>
void CRegistrationDeciderOrOptimizer<GRAPH_t>::setGraphPtr(GRAPH_t* graph) {
	using namespace mrpt::utils;

	m_graph = graph;
	MRPT_LOG_DEBUG_STREAM << "Fetched the graph successfully";
}

#endif /* end of include guard: CREGISTRATIONDECIDEROROPTIMIZER_IMPL_H */
