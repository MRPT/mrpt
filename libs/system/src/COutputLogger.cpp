/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "system-precomp.h"  // Precompiled headers
//
#include <mrpt/core/exceptions.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/filesystem.h>

#include <cstdarg>  // for logFmt
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#ifdef _MSC_VER
#define WIN32_LEAN_AND_MEAN
#include <windows.h>  // OutputDebugString() for MSVC
#endif

using namespace mrpt;
using namespace mrpt::system;
using namespace std;

/**
 * Implementation file for the COutputLogger header class
 */

// COutputLogger
// ////////////////////////////////////////////////////////////

static std::array<mrpt::system::TConsoleColor, NUMBER_OF_VERBOSITY_LEVELS>
	logging_levels_to_colors = {
		CONCOL_BLUE,  // LVL_DEBUG
		CONCOL_NORMAL,  // LVL_INFO
		CONCOL_GREEN,  // LVL_WARN
		CONCOL_RED  // LVL_ERROR
};

std::array<mrpt::system::TConsoleColor, NUMBER_OF_VERBOSITY_LEVELS>&
	COutputLogger::logging_levels_to_colors()
{
	return ::logging_levels_to_colors;
}

static std::array<std::string, NUMBER_OF_VERBOSITY_LEVELS>
	logging_levels_to_names = {
		"DEBUG",  // LVL_DEBUG
		"INFO ",  // LVL_INFO
		"WARN ",  // LVL_WARN
		"ERROR"  // LVL_ERROR
};
std::array<std::string, NUMBER_OF_VERBOSITY_LEVELS>&
	COutputLogger::logging_levels_to_names()
{
	return ::logging_levels_to_names;
}

COutputLogger::~COutputLogger() = default;

void COutputLogger::logStr(
	const VerbosityLevel level, std::string_view msg_str) const
{
	// initialize a TMsg object
	TMsg msg(level, msg_str, *this);
	if (logging_enable_keep_record)
	{
		auto lck = mrpt::lockHelper(*m_historyMtx);
		m_history.push_back(msg);
	}

	if (level >= m_min_verbosity_level && logging_enable_console_output)
	{
		msg.dumpToConsole();

		// User callbacks:
		for (const auto& c : m_listCallbacks)
			c(msg.body, msg.level, msg.name, msg.timestamp);
	}
}

void COutputLogger::logFmt(
	const VerbosityLevel level, const char* fmt, ...) const
{
	// check for nullptr pointer
	if (!fmt) return;

	// initialize the va_list and let generateStringFromFormat do the work
	// http://c-faq.com/varargs/handoff.html
	va_list argp;
	va_start(argp, fmt);
	std::string str = this->generateStringFromFormat(fmt, argp);
	va_end(argp);

	this->logStr(level, str);
}

std::string COutputLogger::generateStringFromFormat(
	std::string_view fmt, va_list argp) const
{
	int result = -1, length = 1024;
	std::vector<char> buffer;
	// make sure that the buffer is large enough to handle the string
	while (result == -1)
	{
		buffer.resize(length + 10);
		result = os::vsnprintf(&buffer[0], length, fmt.data(), argp);

		// http://www.cplusplus.com/reference/cstdio/vsnprintf/
		// only when this returned value is non-negative and less than n, the
		// string has been completely written
		if (result >= length) result = -1;
		length *= 2;
	}

	// return result to the caller
	return std::string(&buffer[0]);
}
void COutputLogger::logCond(
	const VerbosityLevel level, bool cond, const std::string& msg_str) const
{
	if (!cond) return;
	this->logStr(level, msg_str);
}

void COutputLogger::setLoggerName(const std::string& name)
{
	m_logger_name = name;
}

std::string COutputLogger::getLoggerName() const { return m_logger_name; }
void COutputLogger::setMinLoggingLevel(
	const VerbosityLevel level /*= LVL_INFO */)
{
	m_min_verbosity_level = level;
}
void COutputLogger::setVerbosityLevel(const VerbosityLevel level)
{
	m_min_verbosity_level = level;
}

void COutputLogger::getLogAsString(std::string& fname) const
{
	fname.clear();
	auto lck = mrpt::lockHelper(*m_historyMtx);
	for (const auto& h : m_history) fname += h.getAsString();
}
std::string COutputLogger::getLogAsString() const
{
	std::string str;
	this->getLogAsString(str);
	return str;
}
void COutputLogger::writeLogToFile(const std::optional<string> fname_in) const
{
	using namespace std::string_literals;

	// determine the filename - open it
	std::string fname =
		fname_in.has_value()
			? fname_in.value()
			: mrpt::system::fileNameStripInvalidChars(m_logger_name) + ".log"s;

	std::ofstream f(fname);
	ASSERTMSG_(
		f.is_open(), mrpt::format(
						 "[%s] Could not open external file: %s",
						 m_logger_name.c_str(), fname.c_str()));

	std::string hist_str;
	this->getLogAsString(hist_str);
	f << hist_str;
	f.close();
}

void COutputLogger::dumpLogToConsole() const
{
	auto lck = mrpt::lockHelper(*m_historyMtx);
	for (const auto& h : m_history) h.dumpToConsole();
}

std::string COutputLogger::getLoggerLastMsg() const
{
	auto lck = mrpt::lockHelper(*m_historyMtx);
	TMsg last_msg = m_history.back();
	return last_msg.getAsString();
}

void COutputLogger::getLoggerLastMsg(std::string& msg_str) const
{
	msg_str = this->getLoggerLastMsg();
}

void COutputLogger::loggerReset() { *this = COutputLogger(); }

// TMsg Struct
// ////////////////////////////////////////////////////////////

COutputLogger::TMsg::TMsg(
	const mrpt::system::VerbosityLevel in_level, std::string_view msg_str,
	const COutputLogger& logger)
	: timestamp(mrpt::Clock::now()),  // fill with the current time
	  level(in_level),
	  name(logger.getLoggerName()),
	  body(msg_str)
{
}

COutputLogger::TMsg::~TMsg() = default;

std::string COutputLogger::TMsg::getAsString() const
{
	stringstream out;
	out << "[" << mrpt::system::timeLocalToString(timestamp, 4) << "|"
		<< COutputLogger::logging_levels_to_names()[level] << "|" << name
		<< "] " << body;
	if (!body.empty() && *body.rbegin() != '\n') out << std::endl;

	return out.str();
}
void COutputLogger::TMsg::getAsString(std::string* contents) const
{
	*contents = this->getAsString();
}
void COutputLogger::TMsg::writeToStream(std::ostream& out) const
{
	const std::string str = getAsString();
	out << str;
#ifdef _MSC_VER
	OutputDebugStringA(str.c_str());
#endif
}
void COutputLogger::TMsg::dumpToConsole() const
{
	const std::string str = getAsString();

	const bool dump_to_cerr = (level == LVL_ERROR);  // LVL_ERROR alternatively
	// dumped to stderr instead
	// of stdout

	// Set console color:
	const TConsoleColor concol =
		COutputLogger::logging_levels_to_colors()[level];
	mrpt::system::setConsoleColor(concol, dump_to_cerr);
	// Output msg:
	(dump_to_cerr ? std::cerr : std::cout) << str;
	// Switch back to normal color:
	mrpt::system::setConsoleColor(CONCOL_NORMAL);
#ifdef _MSC_VER
	OutputDebugStringA(
		str.c_str());  // call benchmarked: avrg 90 us (50-200 us)
#endif
}

void COutputLogger::logRegisterCallback(output_logger_callback_t userFunc)
{
	m_listCallbacks.emplace_back(userFunc);
}

template <typename T, typename... U>
size_t getAddress(std::function<T(U...)> f)
{
	using fnType = T (*)(U...);
	auto** fnPointer = f.template target<fnType*>();
	return (size_t)*fnPointer;
}

bool COutputLogger::logDeregisterCallback(output_logger_callback_t userFunc)
{
	for (auto it = m_listCallbacks.begin(); it != m_listCallbacks.end(); ++it)
	{
		if (getAddress(*it) == getAddress(userFunc))
		{
			m_listCallbacks.erase(it);
			return true;
		}
	}
	return false;
}
