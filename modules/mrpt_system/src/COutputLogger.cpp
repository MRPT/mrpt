/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

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

static std::array<
    mrpt::system::ConsoleForegroundColor,
    NUMBER_OF_VERBOSITY_LEVELS>
    logging_levels_to_colors = {
        ConsoleForegroundColor::BLUE,     // LVL_DEBUG
        ConsoleForegroundColor::DEFAULT,  // LVL_INFO
        ConsoleForegroundColor::GREEN,    // LVL_WARN
        ConsoleForegroundColor::RED       // LVL_ERROR
};

std::array<mrpt::system::ConsoleForegroundColor, NUMBER_OF_VERBOSITY_LEVELS>&
COutputLogger::logging_levels_to_colors()
{
  return ::logging_levels_to_colors;
}

static std::array<std::string, NUMBER_OF_VERBOSITY_LEVELS> logging_levels_to_names = {
    "DEBUG",  // LVL_DEBUG
    "INFO ",  // LVL_INFO
    "WARN ",  // LVL_WARN
    "ERROR"   // LVL_ERROR
};
std::array<std::string, NUMBER_OF_VERBOSITY_LEVELS>& COutputLogger::logging_levels_to_names()
{
  return ::logging_levels_to_names;
}

COutputLogger::~COutputLogger() = default;

void COutputLogger::logStr(const VerbosityLevel level, std::string_view msg_str) const
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
  }

  // User callbacks:
  if (level >= m_min_verbosity_level_callbacks)
  {
    for (const auto& c : m_listCallbacks)
    {
      c(msg.body, msg.level, msg.name, msg.timestamp);
    }
  }
}

void COutputLogger::logFmt(const VerbosityLevel level, const char* fmt, ...) const
{
  // check for nullptr pointer
  if (!fmt)
  {
    return;
  }

  // initialize the va_list and let generateStringFromFormat do the work
  // http://c-faq.com/varargs/handoff.html
  va_list argp;
  va_start(argp, fmt);
  std::string str = this->generateStringFromFormat(fmt, argp);
  va_end(argp);

  this->logStr(level, str);
}

std::string COutputLogger::generateStringFromFormat(std::string_view fmt, va_list argp) const
{
  // --- Step 1: Determine required buffer size ---

  // Create a copy of the va_list, as vsnprintf might invalidate argp
  va_list argp_copy;
  va_copy(argp_copy, argp);

  // Ensure the format string is null-terminated for vsnprintf
  std::string fmt_null_terminated(fmt);

  // Call vsnprintf with null buffer and zero size to get the needed length
  // The return value is the number of characters required, excluding the null terminator.
  int needed_size = std::vsnprintf(nullptr, 0, fmt_null_terminated.c_str(), argp_copy);

  // Clean up the copied va_list
  va_end(argp_copy);

  // Check for encoding errors or other failures from the size calculation call
  if (needed_size < 0)
  {
    // Handle error:
    return "[vsnprintf size calculation error]";
  }

  // --- Step 2: Format into a correctly sized buffer ---

  // Create a string to act as the buffer.
  // Size is needed_size; std::string handles null termination automatically
  // when accessing via c_str(), and provides buffer space internally.
  std::string buffer(
      static_cast<std::string::size_type>(needed_size), '\0');  // Initialize with null chars

  // Call vsnprintf again, this time with the allocated buffer and the *original* va_list.
  // Pass buffer.size() + 1 for the size argument to include space for the null terminator.
  int result = std::vsnprintf(&buffer[0], buffer.size() + 1, fmt_null_terminated.c_str(), argp);

  // --- Step 3: Final checks and return ---

  // Check for errors during the actual formatting
  if (result < 0)
  {
    return "[vsnprintf formatting error]";
  }

  // Optional sanity check: The number of chars written should match the calculated size
  // Note: result could theoretically be larger than needed_size if va_list state changed
  // unexpectedly between va_copy and the second call, though highly unlikely.
  if (static_cast<size_t>(result) < buffer.size())
  {
    buffer.resize(static_cast<size_t>(result));
  }
  // No need to handle result > needed_size explicitly, as vsnprintf respects the buffer limit.

  // Return the formatted string
  return buffer;
}

void COutputLogger::logCond(const VerbosityLevel level, bool cond, const std::string& msg_str) const
{
  if (!cond)
  {
    return;
  }
  this->logStr(level, msg_str);
}

void COutputLogger::setLoggerName(const std::string& name) { m_logger_name = name; }

std::string COutputLogger::getLoggerName() const { return m_logger_name; }

void COutputLogger::setMinLoggingLevel(const VerbosityLevel level)
{
  m_min_verbosity_level = level;
  m_min_verbosity_level_callbacks = level;
}

void COutputLogger::setVerbosityLevelForCallbacks(const VerbosityLevel level)
{
  m_min_verbosity_level_callbacks = level;
}

void COutputLogger::getLogAsString(std::string& fname) const
{
  fname.clear();
  auto lck = mrpt::lockHelper(*m_historyMtx);
  for (const auto& h : m_history)
  {
    fname += h.getAsString();
  }
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
  std::string fname = fname_in.has_value()
                          ? fname_in.value()
                          : mrpt::system::fileNameStripInvalidChars(m_logger_name) + ".log"s;

  std::ofstream f(fname);
  ASSERTMSG_(
      f.is_open(),
      mrpt::format("[%s] Could not open external file: %s", m_logger_name.c_str(), fname.c_str()));

  std::string hist_str;
  this->getLogAsString(hist_str);
  f << hist_str;
  f.close();
}

void COutputLogger::dumpLogToConsole() const
{
  auto lck = mrpt::lockHelper(*m_historyMtx);
  for (const auto& h : m_history)
  {
    h.dumpToConsole();
  }
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
    const mrpt::system::VerbosityLevel in_level,
    std::string_view msg_str,
    const COutputLogger& logger) :
    timestamp(mrpt::Clock::now()),  // fill with the current time
    level(in_level),
    name(logger.getLoggerName()),
    body(msg_str)
{
}

std::string COutputLogger::TMsg::getAsString() const
{
  stringstream out;
  out << "[" << mrpt::system::timeLocalToString(timestamp, 4) << "|"
      << COutputLogger::logging_levels_to_names()[level] << "|" << name << "] " << body;
  if (!body.empty() && *body.rbegin() != '\n')
  {
    out << std::endl;
  }

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
  const ConsoleForegroundColor concol = COutputLogger::logging_levels_to_colors()[level];
  mrpt::system::consoleColorAndStyle(
      concol, ConsoleBackgroundColor::DEFAULT, ConsoleTextStyle::REGULAR, dump_to_cerr);
  // Output msg:
  (dump_to_cerr ? std::cerr : std::cout) << str;
  // Switch back to normal color:
  mrpt::system::consoleColorAndStyle(
      ConsoleForegroundColor::DEFAULT, ConsoleBackgroundColor::DEFAULT, ConsoleTextStyle::REGULAR,
      dump_to_cerr);
#ifdef _MSC_VER
  OutputDebugStringA(str.c_str());  // call benchmarked: avrg 90 us (50-200 us)
#endif
}

void COutputLogger::logRegisterCallback(output_logger_callback_t userFunc)
{
  m_listCallbacks.emplace_back(userFunc);
}

bool COutputLogger::logDeregisterCallback(output_logger_callback_t userFunc)
{
  for (auto it = m_listCallbacks.begin(); it != m_listCallbacks.end(); ++it)
  {
    if (it->target<void (*)()>() == userFunc.target<void (*)()>())
    {
      m_listCallbacks.erase(it);
      return true;
    }
  }
  return false;
}
