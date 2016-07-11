/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/utils/CStdOutStream.h>
#include "COutputLogger.h"
#include <mrpt/system/threads.h>


using namespace mrpt;
using namespace mrpt::system;
using namespace mrpt::utils;

using namespace std;

/**
 * Implementation file for the COutputLogger header class
 */

// COutputLogger
// ////////////////////////////////////////////////////////////

// variables shared among the various logger instances
size_t COutputLogger_t::m_times_for_log_const = 0;
size_t COutputLogger_t::m_times_for_log_const_thresh = 2; // warn 2 times at most

COutputLogger_t::COutputLogger_t(std::string name) {
	this->reset();
	m_name = name;

}
COutputLogger_t::COutputLogger_t() {
	this->reset();
}
COutputLogger_t::~COutputLogger_t() { }

void COutputLogger_t::log(const std::string& msg_str) {
	if (m_curr_level < m_min_verbosity_level)
		return;

	// initialize a TMsg object
	TMsg msg(msg_str, *this);
	m_history.push_back(msg);

	if (m_print_message_automatically) {
		msg.dumpToConsole();

		// automatically set the color back to normal
		mrpt::system::setConsoleColor(CONCOL_NORMAL);
	}

}
void COutputLogger_t::log(const std::string& msg_str, const VerbosityLevel& level) {
	// temporarily override the logging level
	VerbosityLevel global_level = m_curr_level;
	m_curr_level = level;

	this->log(msg_str);

	m_curr_level = global_level;
}

void COutputLogger_t::log(const std::string& msg_str) const {
	warnForLogConstMethod();

	TMsg msg(msg_str, *this);
	msg.dumpToConsole();
}

void COutputLogger_t::log(const std::string& msg_str, const VerbosityLevel& level) const {
	warnForLogConstMethod();

	TMsg msg(msg_str, *this);
	msg.level = level;
	msg.dumpToConsole();
}

void COutputLogger_t::warnForLogConstMethod() const {
	if (m_times_for_log_const++ >= m_times_for_log_const_thresh) {
		return;
	}

	// warn the user of the method behavior
	std::string msg_str("Using the log method inside a const function/method. Message will not be recorded in COutputLogger_t history");
	TMsg warning_msg(msg_str, *this);
	warning_msg.level = LVL_WARN;
	warning_msg.dumpToConsole();
	mrpt::system::sleep(1000);
}


void COutputLogger_t::logCond(const std::string& msg_str, 
		bool cond) {
	if (cond) {
		this->log(msg_str);
	}
}
void COutputLogger_t::logCond(const std::string& msg_str, 
		const VerbosityLevel& level,
		bool cond) {
	if (cond) {
		this->log(msg_str, level);
	}
}

void COutputLogger_t::setName(const std::string& name) { m_name = name; }

std::string COutputLogger_t::getName() const { return m_name; }

void COutputLogger_t::setLoggingLevel(const VerbosityLevel& level /*= LVL_INFO */) {
	m_curr_level = level;

	if (m_curr_level < m_min_verbosity_level) {
		mrpt::system::setConsoleColor(CONCOL_RED);
		std::cout << format("Current VerbosityLevel provided %d is smaller than the minimum allowed verbosity level %d. Logger will ignore all logging directives\n", m_curr_level, m_min_verbosity_level); 
		mrpt::system::setConsoleColor(CONCOL_NORMAL);
		mrpt::system::sleep(1000);
	}
}

void COutputLogger_t::setMinLoggingLevel(const VerbosityLevel& level /*= LVL_INFO */) {
	m_min_verbosity_level = level;

	if (m_curr_level < m_min_verbosity_level) {
		mrpt::system::setConsoleColor(CONCOL_RED);
		std::cout << format("Current VerbosityLevel %d is smaller than the minimum allowed verbosity level %d.\nLogger will ignore all logging directives using this VerbosityLevel\n", m_curr_level, m_min_verbosity_level); 
		mrpt::system::setConsoleColor(CONCOL_NORMAL);
		mrpt::system::sleep(1000);
	}
}


VerbosityLevel COutputLogger_t::getCurrentLoggingLevel() const { return m_curr_level; }

void COutputLogger_t::writeToFile(const std::string* fname_in /* = NULL */) const {
	// determine the filename - open it
	std::string fname;
	if (fname_in) {
		fname = *fname_in;
	}
	else {
		fname = m_name + ".log";
	}
	CFileOutputStream fstream(fname);
	ASSERTMSG_(fstream.fileOpenCorrectly(),
			mrpt::format("\n[%s:] Could not open external file: %s",
				m_name.c_str(), fname.c_str()) );

	for (std::vector<TMsg>::const_iterator hist_it = m_history.begin();
			hist_it != m_history.end(); ++hist_it) {
		fstream.printf("%s\n", hist_it->getAsString().c_str() );
	}

	fstream.close();
}

void COutputLogger_t::dumpToConsole() const{
	for (std::vector<TMsg>::const_iterator hist_it = m_history.begin();
			hist_it != m_history.end(); ++hist_it) {
		hist_it->dumpToConsole();
	}
}

std::string COutputLogger_t::getLastMsg() const {
	TMsg last_msg = m_history.back();
	return last_msg.getAsString();
}

void COutputLogger_t::getLastMsg(std::string* msg_str) const {
	*msg_str = this->getLastMsg();
}


void COutputLogger_t::reset() {
	m_name = "Logger"; // just the default name

	m_history.clear();
	m_curr_level = LVL_INFO;
	m_curr_timestamp = INVALID_TIMESTAMP;

	m_print_message_automatically = true;

	// set the minimum logging level allowed for printing. By default its
	// LVL_DEBUG
	m_min_verbosity_level = LVL_DEBUG;

}

// TMsg Struct
// ////////////////////////////////////////////////////////////

COutputLogger_t::TMsg::TMsg(const std::string& msg_str, const COutputLogger_t& logger) {
	this->reset();

	name = logger.getName();
	level = logger.getCurrentLoggingLevel();
	timestamp = mrpt::system::now(); // fill with the current time
	body = msg_str;
}
COutputLogger_t::TMsg::~TMsg() { }

mrpt::system::TConsoleColor COutputLogger_t::TMsg::getConsoleColor(VerbosityLevel level) const {
	TConsoleColor color;

  map<VerbosityLevel, mrpt::system::TConsoleColor>::const_iterator search;
  search = m_levels_to_colors.find(level);

	// if found return the corresponding color, otherwise return normal...
  if (search != m_levels_to_colors.end()) {
  	color = search->second;
  }
  else {
  	color = CONCOL_NORMAL;
  }

	return color;
}

std::string COutputLogger_t::TMsg::getLoggingLevelName(VerbosityLevel level) const {
	std::string name;

  map<VerbosityLevel, std::string>::const_iterator search;
  search = m_levels_to_names.find(level);

	// if found return the corresponding color, otherwise return normal...
  if (search != m_levels_to_names.end()) {
  	name = search->second;
  }
  else {
  	name = "UNKNOWN_LVL";
  }

	return name;
}

void COutputLogger_t::TMsg::reset() {
	timestamp = INVALID_TIMESTAMP;
	level = LVL_INFO;
	name = "Message"; // default name
	body.clear(); 

	m_levels_to_colors.clear();
	m_levels_to_colors[LVL_DEBUG] = CONCOL_BLUE;
	m_levels_to_colors[LVL_INFO]  = CONCOL_NORMAL;
	m_levels_to_colors[LVL_WARN]  = CONCOL_GREEN;
	m_levels_to_colors[LVL_ERROR] = CONCOL_RED;

	m_levels_to_names.clear();
 	m_levels_to_names[LVL_DEBUG] = "DEBUG";
	m_levels_to_names[LVL_INFO]  = "INFO ";
	m_levels_to_names[LVL_WARN]  = "WARN ";
	m_levels_to_names[LVL_ERROR] = "ERROR";
}

std::string COutputLogger_t::TMsg::getAsString() const {
	stringstream out;
	out.str("");
	out << "[" << name <<  " | " << this->getLoggingLevelName(level) << " | " 
		<< mrpt::system::timeToString(timestamp) 
		<< "] " << body;

	return out.str();
}
void COutputLogger_t::TMsg::getAsString(std::string* contents) const {
	*contents = this->getAsString();
}
void COutputLogger_t::TMsg::writeToStream(mrpt::utils::CStream& out) const {
	out.printf("%s\n", getAsString().c_str());
}
void COutputLogger_t::TMsg::dumpToConsole() const {
	CStdOutStream mrpt_cout;
	setConsoleColor(this->getConsoleColor(level));
	this->writeToStream(mrpt_cout);
	
}
