/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <mrpt/core/lock_helper.h>
#include <mrpt/io/CCompressedOutputStream.h>
#include <mrpt/nav/reactive/NavigationLogger.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>

using namespace mrpt::nav;
using namespace mrpt::io;

void NavigationLogger::enableLogFile(
    bool enable, const std::string& logDir, mrpt::system::COutputLogger* logger)
{
  try
  {
    if (!enable)
    {
      if (m_logFile)
      {
        if (logger)
          logger->logStr(
              mrpt::system::LVL_DEBUG, "[NavigationLogger::enableLogFile] Stopping logging.");
        m_logFile.reset();
      }
      return;
    }

    // Enable
    if (m_logFile) return;  // Already enabled

    const std::string& dir = logDir.empty() ? m_navlogfiles_dir : logDir;
    if (!logDir.empty()) m_navlogfiles_dir = logDir;

    if (logger)
      logger->logStr(
          mrpt::system::LVL_DEBUG,
          mrpt::format(
              "[NavigationLogger::enableLogFile] Creating rnav log directory: %s", dir.c_str()));

    mrpt::system::createDirectory(dir);
    if (!mrpt::system::directoryExists(dir))
    {
      THROW_EXCEPTION_FMT("Could not create directory for navigation logs: `%s`", dir.c_str());
    }

    std::string filToOpen;
    for (unsigned int nFile = 0;; nFile++)
    {
      filToOpen = mrpt::format("%s/log_%03u.reactivenavlog", dir.c_str(), nFile);
      if (!mrpt::system::fileExists(filToOpen)) break;
    }

    auto fil = std::make_unique<CCompressedOutputStream>();
    if (!fil->open(filToOpen))
    {
      THROW_EXCEPTION_FMT("Error opening log file: `%s`", filToOpen.c_str());
    }
    m_logFile = std::move(fil);

    if (logger)
      logger->logStr(
          mrpt::system::LVL_DEBUG,
          mrpt::format(
              "[NavigationLogger::enableLogFile] Logging to file `%s`", filToOpen.c_str()));
  }
  catch (const std::exception& e)
  {
    if (logger)
      logger->logStr(
          mrpt::system::LVL_ERROR,
          mrpt::format("[NavigationLogger::enableLogFile] Exception: %s", e.what()));
  }
}

void NavigationLogger::getLastLogRecord(CLogFileRecord& o)
{
  auto lck = mrpt::lockHelper(m_critZoneLastLog);
  o = m_lastLogRecord;
}

void NavigationLogger::writeLogRecord(
    const CLogFileRecord& rec, mrpt::system::CTimeLogger& timelogger)
{
  {
    mrpt::system::CTimeLoggerEntry tle(timelogger, "navigationStep.write_log_file");
    if (m_logFile) mrpt::serialization::archiveFrom(*m_logFile) << rec;
  }
  {
    auto lck = mrpt::lockHelper(m_critZoneLastLog);
    m_lastLogRecord = rec;
  }
}

void NavigationLogger::close()
{
  m_logFile.reset();
  m_prev_logfile = nullptr;
}
