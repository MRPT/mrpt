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
#pragma once

#include <mrpt/io/CStream.h>
#include <mrpt/nav/reactive/CLogFileRecord.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/system/filesystem.h>

#include <memory>
#include <mutex>
#include <string>

namespace mrpt::nav
{
/** Manages navigation log file writing and in-memory log record keeping.
 *
 * Extracted from CAbstractPTGBasedReactive to reduce its complexity.
 * \ingroup nav_reactive
 */
class NavigationLogger
{
 public:
  NavigationLogger() = default;

  /** Enable/disable log file output.
   * \param enable If true, opens a new log file; if false, closes any open file.
   * \param logDir Directory for log files.
   * \param logger Optional COutputLogger for debug messages.
   */
  void enableLogFile(bool enable, const std::string& logDir, mrpt::system::COutputLogger* logger);

  /** Whether log file or in-memory records should be filled */
  [[nodiscard]] bool shouldFillLogRecord() const { return m_logFile || m_enableKeepLogRecords; }

  /** Enables keeping an internal registry of navigation logs */
  void enableKeepLogRecords(bool enable = true) { m_enableKeepLogRecords = enable; }

  /** Provides a copy of the last log record */
  void getLastLogRecord(CLogFileRecord& o);

  /** Changes the prefix for new log files. */
  void setLogFileDirectory(const std::string& sDir) { m_navlogfiles_dir = sDir; }
  [[nodiscard]] std::string getLogFileDirectory() const { return m_navlogfiles_dir; }

  /** Write the completed log record to file and/or store as last record */
  void writeLogRecord(const CLogFileRecord& rec, mrpt::system::CTimeLogger& timelogger);

  /** Access to the raw log stream (for introductory blocks, etc.) */
  [[nodiscard]] mrpt::io::CStream* getLogStream() { return m_logFile.get(); }

  /** Returns true if this is the first time we write to a new log file
   * (used for writing PTG introductory data) */
  [[nodiscard]] bool isNewLogFile() const { return m_logFile && m_logFile.get() != m_prev_logfile; }

  /** Mark the current log file as "already introduced" */
  void markLogFileIntroduced() { m_prev_logfile = m_logFile.get(); }

  /** Close log file and reset state */
  void close();

 private:
  std::unique_ptr<mrpt::io::CStream> m_logFile;
  mrpt::io::CStream* m_prev_logfile{nullptr};
  bool m_enableKeepLogRecords{false};
  CLogFileRecord m_lastLogRecord;
  std::recursive_mutex m_critZoneLastLog;
  std::string m_navlogfiles_dir{"./reactivenav.logs"};
};

}  // namespace mrpt::nav
