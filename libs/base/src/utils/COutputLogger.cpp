/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/utils/CStream.h>
#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/utils/COutputLogger.h>
#include <mrpt/system/threads.h>
#include <cstdio>
#include <sstream>
#include <iostream>
#include <cstdarg> // for logFmt

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

mrpt::system::TConsoleColor COutputLogger::logging_levels_to_colors[NUMBER_OF_VERBOSITY_LEVELS] = {
	CONCOL_BLUE,  // LVL_DEBUG
	CONCOL_NORMAL, // LVL_INFO
	CONCOL_GREEN, // LVL_WARN
	CONCOL_RED // LVL_ERROR
};

std::string COutputLogger::logging_levels_to_names[NUMBER_OF_VERBOSITY_LEVELS] = {
 	"DEBUG", //LVL_DEBUG
	"INFO ", // LVL_INFO
	"WARN ", // LVL_WARN
	"ERROR" // LVL_ERROR
};

COutputLogger::COutputLogger(const std::string &name) {
	this->loggerReset();
	m_logger_name = name;

}
COutputLogger::COutputLogger() {
	this->loggerReset();
}
COutputLogger::~COutputLogger() { }

void COutputLogger::logStr(const VerbosityLevel level, const std::string& msg_str) const {
	if (level<m_min_verbosity_level)
		return;

	// initialize a TMsg object
	TMsg msg(level, msg_str, *this);
	if (logging_enable_keep_record)
		m_history.push_back(msg);

	if (logging_enable_console_output) {
		msg.dumpToConsole();
	}

	// User callbacks:
	for (const auto &c : m_listCallbacks)
		(*c.func)(msg.body,msg.level,msg.name,msg.timestamp,c.userParam);
}

void COutputLogger::logFmt(const VerbosityLevel level, const char* fmt, ...) const {
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

	this->logStr(level,str);
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
void COutputLogger::logCond(const VerbosityLevel level, bool cond, const std::string& msg_str) const
{
	if (!cond) return;
	this->logStr(level,msg_str);
}

void COutputLogger::setLoggerName(const std::string& name) { m_logger_name = name; }

std::string COutputLogger::getLoggerName() const { return m_logger_name; }

void COutputLogger::setMinLoggingLevel(const VerbosityLevel level /*= LVL_INFO */) {
	m_min_verbosity_level = level;
}
void COutputLogger::setVerbosityLevel(const VerbosityLevel level) {
	m_min_verbosity_level = level;
}

void COutputLogger::getLogAsString(std::string& fname) const {
	fname.clear();
	for (const auto & h : m_history)
		fname += h.getAsString();
}
std::string COutputLogger::getLogAsString() const{
	std::string str;
	this->getLogAsString(str);
	return str;
}
void COutputLogger::writeLogToFile(const std::string* fname_in /* = NULL */) const {
	// determine the filename - open it
	std::string fname;
	if (fname_in) {
		fname = *fname_in;
	}
	else {
		fname = m_logger_name + ".log";
	}
	CFileOutputStream fstream(fname);
	ASSERTMSG_(fstream.fileOpenCorrectly(),
			mrpt::format("\n[%s:] Could not open external file: %s",
				m_logger_name.c_str(), fname.c_str()) );

	std::string hist_str;
	this->getLogAsString(hist_str);
	fstream.printf("%s", hist_str.c_str());
	fstream.close();
}

void COutputLogger::dumpLogToConsole() const{
	for (const auto &h :m_history)
		h.dumpToConsole();
}

std::string COutputLogger::getLoggerLastMsg() const {
	TMsg last_msg = m_history.back();
	return last_msg.getAsString();
}

void COutputLogger::getLoggerLastMsg(std::string& msg_str) const {
	msg_str = this->getLoggerLastMsg();
}


void COutputLogger::loggerReset() {
	m_logger_name = "log"; // just the default name

	m_history.clear();
	logging_enable_console_output = true;
	logging_enable_keep_record    = false;

	// set the minimum logging level allowed for printing. By default its
	// LVL_INFO
	m_min_verbosity_level = LVL_INFO;
}

// TMsg Struct
// ////////////////////////////////////////////////////////////

COutputLogger::TMsg::TMsg(const mrpt::utils::VerbosityLevel level, const std::string& msg_str, const COutputLogger& logger) {
	this->reset();

	name = logger.getLoggerName();
	this->level = level;
	timestamp = mrpt::system::getCurrentTime(); // fill with the current time
	body = msg_str;
}
COutputLogger::TMsg::~TMsg() { }

void COutputLogger::TMsg::reset() {
	timestamp = INVALID_TIMESTAMP;
	level = LVL_INFO;
	name = "Message"; // default name
	body.clear();
}

std::string COutputLogger::TMsg::getAsString() const {
	stringstream out;
	out.str("");
	out << "[" << name <<  "|" << COutputLogger::logging_levels_to_names[level]  << "|"
	<< mrpt::system::timeLocalToString(timestamp,4) 
		<< "] " << body;
	if (!body.empty() && *body.rbegin()!='\n')
		out<<std::endl;

	return out.str();
}
void COutputLogger::TMsg::getAsString(std::string* contents) const {
	*contents = this->getAsString();
}
void COutputLogger::TMsg::writeToStream(mrpt::utils::CStream& out) const {
	const std::string str = getAsString();
	out.printf("%s", str.c_str());
#ifdef _MSC_VER
	OutputDebugStringA(str.c_str());
#endif
}
void COutputLogger::TMsg::dumpToConsole() const {
	const std::string str = getAsString();

	const bool dump_to_cerr = (level==LVL_ERROR); // LVL_ERROR alternatively dumped to stderr instead of stdout

	// Set console color:
	const TConsoleColor concol = COutputLogger::logging_levels_to_colors[level];
	mrpt::system::setConsoleColor(concol, dump_to_cerr);
	// Output msg:
	(dump_to_cerr ? std::cerr : std::cout) << str;
	// Switch back to normal color:
	mrpt::system::setConsoleColor(CONCOL_NORMAL);
#ifdef _MSC_VER
	OutputDebugStringA(str.c_str());
#endif
}

void COutputLogger::logRegisterCallback(output_logger_callback_t  userFunc, void *userParam )
{
	ASSERT_(userFunc!=NULL);
	TCallbackEntry cbe;
	cbe.func = userFunc;
	cbe.userParam = userParam;
	m_listCallbacks.insert(cbe);
}

void COutputLogger::logDeregisterCallback(output_logger_callback_t  userFunc, void *userParam )
{
	ASSERT_(userFunc!=NULL);
	TCallbackEntry cbe;
	cbe.func = userFunc;
	cbe.userParam = userParam;
	m_listCallbacks.erase(cbe);
}
