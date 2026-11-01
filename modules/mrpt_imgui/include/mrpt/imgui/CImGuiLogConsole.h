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

#include <mrpt/core/Clock.h>
#include <mrpt/imgui/CImGuiFileDialog.h>
#include <mrpt/system/COutputLogger.h>

#include <atomic>
#include <cstdint>
#include <deque>
#include <mutex>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace mrpt::imgui
{
/** A dockable log console: a rolling buffer of messages, e.g. from any
 * number of mrpt::system::COutputLogger objects, with filters by minimum
 * level, source and text, autoscroll, and saving to a file.
 *
 * Messages can be pushed from any thread; render() must be called from the
 * GUI thread.
 *
 * \code
 *   mrpt::imgui::CImGuiLogConsole console;
 *   console.attach(myAlgorithm);  // any COutputLogger
 *   // In a window:
 *   console.render();
 * \endcode
 * \ingroup mrpt_imgui_grp
 */
class CImGuiLogConsole
{
 public:
  struct Entry
  {
    mrpt::Clock::time_point timestamp;
    mrpt::system::VerbosityLevel level = mrpt::system::LVL_INFO;
    std::string source;
    std::string text;
  };

  struct Filter
  {
    mrpt::system::VerbosityLevel minLevel = mrpt::system::LVL_DEBUG;
    /** Case-insensitive substring of the message text or its source. */
    std::string text;
    /** Sources whose messages are hidden. */
    std::set<std::string> hiddenSources;

    [[nodiscard]] bool matches(const Entry& e) const;
  };

  CImGuiLogConsole() = default;
  /** Detaches from all the loggers attached with attach(). */
  ~CImGuiLogConsole();

  CImGuiLogConsole(const CImGuiLogConsole&) = delete;
  CImGuiLogConsole& operator=(const CImGuiLogConsole&) = delete;

  /** Adds a message (thread-safe). Multi-line text becomes one entry per
   * line; a trailing newline is ignored. */
  void push(
      mrpt::system::VerbosityLevel level,
      const std::string& source,
      const std::string& text,
      mrpt::Clock::time_point timestamp = mrpt::Clock::now());

  /** Receives all messages of a logger, from now on. The logger must outlive
   * this object, or be detached first. */
  void attach(mrpt::system::COutputLogger& logger);
  void detach(mrpt::system::COutputLogger& logger);

  void clear();

  /** Oldest messages are dropped beyond this count. */
  std::atomic<size_t> maxEntries{20000};

  Filter filter;
  bool autoScroll = true;
  bool showTimestamps = true;
  bool showSources = true;

  [[nodiscard]] size_t size() const;
  /** Incremented on every change, for cheap change detection. */
  [[nodiscard]] uint64_t version() const { return m_version.load(); }

  /** All the sources seen so far. */
  [[nodiscard]] std::vector<std::string> sources() const;

  /** A copy of the entries that pass the filter. */
  [[nodiscard]] std::vector<Entry> filtered() const;

  /** The entries that pass the filter, as text, one per line. */
  [[nodiscard]] std::string filteredAsText() const;

  /** Formats one entry as a text line. */
  [[nodiscard]] std::string format(const Entry& e) const;

  /** Draws the filter toolbar and the messages in the current window. */
  void render();

 private:
  mutable std::mutex m_mtx;
  std::deque<Entry> m_entries;
  std::set<std::string> m_sources;
  std::atomic<uint64_t> m_version{0};
  std::vector<std::pair<mrpt::system::COutputLogger*, mrpt::system::output_logger_callback_id_t>>
      m_attached;
  CImGuiFileDialog m_saveDialog;

  // Cached filtered view, rebuilt only when entries or filters change:
  uint64_t m_cacheVersion = UINT64_MAX;
  std::string m_cacheFilterKey;
  std::vector<Entry> m_cache;
};

}  // namespace mrpt::imgui
