/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/utils/COutputLogger.h>
#include <mrpt/system/threads.h>

#include <vector>
#include <cstdio>

#ifdef _MSC_VER
	#define WIN32_LEAN_AND_MEAN
	#include <windows.h>   // OutputDebugString() for MSVC
#endif


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
size_t COutputLogger::m_times_for_log_const = 0;
size_t COutputLogger::m_times_for_log_const_thresh = 2; // warn 2 times at most

COutputLogger::COutputLogger(std::string name) {
	this->reset();
	m_name = name;

}
COutputLogger::COutputLogger() {
	this->reset();
}
COutputLogger::~COutputLogger() { }

void COutputLogger::log(const std::string& msg_str) {
	if (m_curr_level < m_min_verbosity_level)
		return;

	// initialize a TMsg object
	TMsg msg(msg_str, *this);
	m_history.push_back(msg);

	if (print_message_automatically) {
		msg.dumpToConsole();

		// automatically set the color back to normal
		mrpt::system::setConsoleColor(CONCOL_NORMAL);
	}

}
void COutputLogger::log(const std::string& msg_str) const {
	warnForLogConstMethod();

	TMsg msg(msg_str, *this);
	msg.dumpToConsole();
}

void COutputLogger::log(const std::string& msg_str, const VerbosityLevel& level) {
	// temporarily override the logging level
	VerbosityLevel global_level = m_curr_level;
	m_curr_level = level;

	this->log(msg_str);

	m_curr_level = global_level;
}
void COutputLogger::log(const std::string& msg_str, const VerbosityLevel& level) const {
	warnForLogConstMethod();

	TMsg msg(msg_str, *this);
	msg.level = level;
	msg.dumpToConsole();
}

void COutputLogger::logFmt(const char* fmt, ...) {
	// see MRPT/libs/base/src/utils/CDeugOutputCapable.cpp for the iniitial
	// implementtion

	// check for NULL pointer
	if (!fmt) return;

	// initialize the va_list and let generateStringFromFormat do the work
	// http://c-faq.com/varargs/handoff.html
	va_list argp;
	va_start(argp, fmt);
	std::string str = this->generateStringFromFormat(fmt, argp);
	va_end(argp);

	this->log(str);
}

void COutputLogger::logFmt(const char* fmt, ...) const {
	if (!fmt) return;

	va_list argp;
	va_start(argp, fmt);
	std::string str = this->generateStringFromFormat(fmt, argp);
	va_end(argp);

	this->log(str);
}

std::string COutputLogger::generateStringFromFormat(const char* fmt, va_list argp) const{
	int   result = -1, length = 1024;
	std::vector<char> buffer;
	// make sure that the buffer is large enough to handle the string
	while (result == -1)
	{
		buffer.resize(length + 10);
		result = os::vsnprintf(&buffer[0], length, fmt, argp);

		// http://www.cplusplus.com/reference/cstdio/vsnprintf/
		// only when this returned value is non-negative and less than n, the
		// string has been completely written
		if (result>=length) result=-1;
		length*=2;
	}

	// return result to the caller
	return std::string(&buffer[0]);

}
void COutputLogger::logCond(const std::string& msg_str, 
		bool cond) {
	if (cond) {
		this->log(msg_str);
	}
}
void COutputLogger::logCond(const std::string& msg_str, 
		bool cond) const {
	if (cond) {
		this->log(msg_str);
	}
}
void COutputLogger::logCond(const std::string& msg_str, 
		const VerbosityLevel& level,
		bool cond) {
	if (cond) {
		this->log(msg_str, level);
	}
}
void COutputLogger::logCond(const std::string& msg_str, 
		const VerbosityLevel& level,
		bool cond) const {
	if (cond) {
		this->log(msg_str, level);
	}
}

void COutputLogger::warnForLogConstMethod() const {
	if (m_times_for_log_const++ >= m_times_for_log_const_thresh) {
		return;
	}

	// warn the user of the method behavior
	std::string msg_str("Using the log method inside a const function/method. Message will not be recorded in COutputLogger history");
	TMsg warning_msg(msg_str, *this);
	warning_msg.level = LVL_WARN;
	warning_msg.dumpToConsole();
	mrpt::system::sleep(1000);
}

void COutputLogger::setName(const std::string& name) { m_name = name; }

std::string COutputLogger::getName() const { return m_name; }

void COutputLogger::setLoggingLevel(const VerbosityLevel& level /*= LVL_INFO */) {
	m_curr_level = level;

	if (m_curr_level < m_min_verbosity_level) {
		mrpt::system::setConsoleColor(CONCOL_RED);
		std::cout << format("Current VerbosityLevel provided %d is smaller than the minimum allowed verbosity level %d. Logger will ignore all logging directives\n", m_curr_level, m_min_verbosity_level); 
		mrpt::system::setConsoleColor(CONCOL_NORMAL);
		mrpt::system::sleep(1000);
	}
}

void COutputLogger::setMinLoggingLevel(const VerbosityLevel& level /*= LVL_INFO */) {
	m_min_verbosity_level = level;

	if (m_curr_level < m_min_verbosity_level) {
		mrpt::system::setConsoleColor(CONCOL_RED);
		std::cout << format("Current VerbosityLevel %d is smaller than the minimum allowed verbosity level %d.\nLogger will ignore all logging directives using this VerbosityLevel\n", m_curr_level, m_min_verbosity_level); 
		mrpt::system::setConsoleColor(CONCOL_NORMAL);
		mrpt::system::sleep(1000);
	}
}


VerbosityLevel COutputLogger::getLoggingLevel() const { return m_curr_level; }

void COutputLogger::getAsString(std::string* fname) const {
	for (std::vector<TMsg>::const_iterator hist_it = m_history.begin();
			hist_it != m_history.end(); ++hist_it) {
		*fname += hist_it->getAsString() + "\n";
	}
}
std::string COutputLogger::getAsString() const{
	std::string str;
	this->getAsString(&str);

	return str;
}
void COutputLogger::writeToFile(const std::string* fname_in /* = NULL */) const {
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

	std::string hist_str;
	this->getAsString(&hist_str);
	fstream.printf("%s\n", hist_str.c_str());

	fstream.close();
}

void COutputLogger::dumpToConsole() const{
	for (std::vector<TMsg>::const_iterator hist_it = m_history.begin();
			hist_it != m_history.end(); ++hist_it) {
		hist_it->dumpToConsole();
	}
}

std::string COutputLogger::getLastMsg() const {
	TMsg last_msg = m_history.back();
	return last_msg.getAsString();
}

void COutputLogger::getLastMsg(std::string* msg_str) const {
	*msg_str = this->getLastMsg();
}


void COutputLogger::reset() {
	m_name = "Logger"; // just the default name

	m_history.clear();
	m_curr_level = LVL_INFO;

	print_message_automatically = true;

	// set the minimum logging level allowed for printing. By default its
	// LVL_DEBUG
	m_min_verbosity_level = LVL_DEBUG;

}

// TMsg Struct
// ////////////////////////////////////////////////////////////

COutputLogger::TMsg::TMsg(const std::string& msg_str, const COutputLogger& logger) {
	this->reset();

	name = logger.getName();
	level = logger.getLoggingLevel();
	timestamp = mrpt::system::getCurrentTime(); // fill with the current time
	body = msg_str;
}
COutputLogger::TMsg::~TMsg() { }

mrpt::system::TConsoleColor COutputLogger::TMsg::getConsoleColor(VerbosityLevel level) const {
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

std::string COutputLogger::TMsg::getLoggingLevelName(VerbosityLevel level) const {
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

void COutputLogger::TMsg::reset() {
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

std::string COutputLogger::TMsg::getAsString() const {
	stringstream out;
	out.str("");
	out << "[" << name <<  " | " << this->getLoggingLevelName(level) << " | " 
		<< mrpt::system::timeToString(timestamp) 
		<< "] " << body;

	return out.str();
}
void COutputLogger::TMsg::getAsString(std::string* contents) const {
	*contents = this->getAsString();
}
void COutputLogger::TMsg::writeToStream(mrpt::utils::CStream& out) const {
	const std::string str = getAsString();
	out.printf("%s\n", str.c_str());
#ifdef _MSC_VER
	OutputDebugStringA(str.c_str());
#endif
}
void COutputLogger::TMsg::dumpToConsole() const {
	const std::string str = getAsString() + "\n";

	const bool dump_to_cerr = false; // TO-DO: Allow LVL_ERROR to be alternatively dumped to stderr instead of stdout??

	mrpt::system::setConsoleColor(this->getConsoleColor(level), dump_to_cerr);
	::fputs(str.c_str(), dump_to_cerr ? stderr:stdout );
#ifdef _MSC_VER
	OutputDebugStringA(str.c_str());
#endif
}

