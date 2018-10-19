/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

namespace mrpt::graphslam
{
template <class GRAPH_T>
const std::string CRegistrationDeciderOrOptimizer<GRAPH_T>::header_sep =
	std::string(80, '-');
template <class GRAPH_T>
const std::string CRegistrationDeciderOrOptimizer<GRAPH_T>::report_sep =
	std::string(2, '\n');

template <class GRAPH_T>
void CRegistrationDeciderOrOptimizer<GRAPH_T>::initializeLoggers(
	const std::string& name)
{
	this->setClassName(name);  // all the names in one call
	this->logging_enable_keep_record = true;

	// just for the messages until reading the actual verbosity level, set it to
	// debug.
	this->setMinLoggingLevel(mrpt::system::LVL_DEBUG);
	MRPT_LOG_DEBUG_STREAM("Initialized time, output logger instances.");
}

template <class GRAPH_T>
void CRegistrationDeciderOrOptimizer<GRAPH_T>::setClassName(
	const std::string& name)
{
	this->m_class_name = name;
	this->m_time_logger.setName(this->m_class_name);
	this->setLoggerName(this->m_class_name);
}

template <class GRAPH_T>
void CRegistrationDeciderOrOptimizer<GRAPH_T>::setWindowManagerPtr(
	mrpt::graphslam::CWindowManager* win_manager)
{
	ASSERTDEB_(win_manager);
	m_win_manager = win_manager;

	if (m_win_manager)
	{
		m_win = m_win_manager->win;
		m_win_observer = m_win_manager->observer;
	}
}

template <class GRAPH_T>
void CRegistrationDeciderOrOptimizer<GRAPH_T>::setCriticalSectionPtr(
	std::mutex* graph_section)
{
	m_graph_section = graph_section;
	this->logFmt(
		mrpt::system::LVL_DEBUG, "Fetched the CCRiticalSection successfully");
}

template <class GRAPH_T>
void CRegistrationDeciderOrOptimizer<GRAPH_T>::initializeVisuals()
{
	this->assertVisualsVars();
	m_initialized_visuals = true;
}

template <class GRAPH_T>
void CRegistrationDeciderOrOptimizer<GRAPH_T>::assertVisualsVars()
{
	ASSERTDEBMSG_(this->m_win, "No CDisplayWindow3D* was provided");
	ASSERTDEBMSG_(this->m_win_manager, "No CWindowManager* was provided");
	ASSERTDEBMSG_(this->m_win_observer, "No CWindowObserver* was provided");
}

template <class GRAPH_T>
void CRegistrationDeciderOrOptimizer<GRAPH_T>::updateVisuals()
{
	ASSERTDEB_(m_initialized_visuals);
}

template <class GRAPH_T>
void CRegistrationDeciderOrOptimizer<GRAPH_T>::notifyOfWindowEvents(
	const std::map<std::string, bool>& events_occurred)
{
	ASSERTDEB_(m_initialized_visuals);
}

template <class GRAPH_T>
void CRegistrationDeciderOrOptimizer<GRAPH_T>::loadParams(
	const std::string& source_fname)
{
	MRPT_LOG_DEBUG_STREAM("Loading corresponding parameters");
}

template <class GRAPH_T>
void CRegistrationDeciderOrOptimizer<GRAPH_T>::printParams() const
{
	MRPT_LOG_DEBUG_STREAM("Printing corresponding parameters");
}

template <class GRAPH_T>
void CRegistrationDeciderOrOptimizer<GRAPH_T>::getDescriptiveReport(
	std::string* report_str) const
{
	MRPT_LOG_DEBUG_STREAM("Generating corresponding report");
	// TODO - give the compact form here!
}

template <class GRAPH_T>
void CRegistrationDeciderOrOptimizer<GRAPH_T>::setGraphPtr(GRAPH_T* graph)
{
	m_graph = graph;
	MRPT_LOG_DEBUG_STREAM("Fetched the graph pointer successfully");
}

template <class GRAPH_T>
bool CRegistrationDeciderOrOptimizer<GRAPH_T>::isMultiRobotSlamClass()
{
	return is_mr_slam_class;
}
}  // namespace mrpt::graphslam
