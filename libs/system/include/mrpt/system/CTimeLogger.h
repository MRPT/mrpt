/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/containers/ts_hash_map.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/CTicTac.h>
#include <deque>
#include <map>
#include <optional>
#include <stack>
#include <vector>

namespace mrpt::system
{
/** A versatile "profiler" that logs the time spent within each pair of calls to
 * enter(X)-leave(X), among other stats.
 *  The results can be dumped to cout or to Visual Studio's output panel.
 * This class can be also used to monitorize min/mean/max/total stats of any
 * user-provided parameters via the method CTimeLogger::registerUserMeasure().
 *
 * Optional recording of **all** data can be enabled via
 * enableKeepWholeHistory() (use with caution!).
 *
 * Cost of the profiler itself (measured on MSVC2015, Windows 10, Intel i5-2310
 * 2.9GHz):
 * - `enter()`: average 445 ns
 * - `leave()`: average 316 ns
 *
 *  Recursive methods are supported with no problems, that is, calling "enter(X)
 * enter(X) ... leave(X) leave(X)".
 *  `enter()`/`leave()` are thread-safe, in the sense of they being safe to be
 * called from different threads. However, calling `enter()`/`leave()` for the
 * same user-supplied "section name", from different threads, is not allowed. In
 * the latter case (and, actually, in general since it's safer against
 * exceptions), use the RAII helper class CTimeLoggerEntry.
 *
 * \sa CTimeLoggerEntry
 *
 * \note The default behavior is dumping all the information at destruction.
 * \ingroup mrpt_system_grp
 */
class CTimeLogger : public mrpt::system::COutputLogger
{
   private:
	CTicTac m_tictac;
	bool m_enabled;
	std::string m_name;
	bool m_keep_whole_history{false};

	//! Data of all the calls:
	struct TCallData
	{
		TCallData();

		size_t n_calls{0};
		double min_t{0}, max_t{0}, mean_t{0}, last_t{0};
		std::stack<double, std::vector<double>> open_calls;
		bool has_time_units{true};
		std::optional<std::deque<double>> whole_history{};
	};

   protected:
	constexpr static unsigned int HASH_SIZE_IN_BYTES = 1;
	constexpr static unsigned int HASH_ALLOWED_COLLISIONS = 10;
	// Note: we CANNOT store a std::string_view here due to literals life scope.
	using TDataMap = mrpt::containers::ts_hash_map<
		std::string, TCallData, HASH_SIZE_IN_BYTES, HASH_ALLOWED_COLLISIONS>;

	TDataMap m_data;

	void do_enter(const std::string_view& func_name) noexcept;
	double do_leave(const std::string_view& func_name) noexcept;

   public:
	/** Data of each call section: # of calls, minimum, maximum, average and
	 * overall execution time (in seconds) \sa getStats */
	struct TCallStats
	{
		size_t n_calls{0};
		double min_t{0}, max_t{0}, mean_t{0}, total_t{0}, last_t{0};
	};

	CTimeLogger(
		bool enabled = true, const std::string& name = "",
		const bool keep_whole_history = false);
	/** Destructor */
	~CTimeLogger() override;

	// We must define these 4 because of the definition of a virtual dtor
	// (compiler will not generate the defaults)
	CTimeLogger(const CTimeLogger& o) = default;
	CTimeLogger& operator=(const CTimeLogger& o) = default;
	CTimeLogger(CTimeLogger&& o) = default;
	CTimeLogger& operator=(CTimeLogger&& o) = default;

	/** Dump all stats to a multi-line text string. \sa dumpAllStats,
	 * saveToCVSFile */
	std::string getStatsAsText(const size_t column_width = 80) const;
	/** Returns all the current stats as a map: section_name => stats. \sa
	 * getStatsAsText, dumpAllStats, saveToCVSFile */
	void getStats(std::map<std::string, TCallStats>& out_stats) const;
	/** Dump all stats through the COutputLogger interface. \sa getStatsAsText,
	 * saveToCVSFile */
	void dumpAllStats(const size_t column_width = 80) const;
	/** Resets all stats. By default (deep_clear=false), all section names are
	 * remembered (not freed) so the cost of creating upon the first next call
	 * is avoided. */
	void clear(bool deep_clear = false);
	void enable(bool enabled = true) { m_enabled = enabled; }
	void disable() { m_enabled = false; }
	bool isEnabled() const { return m_enabled; }

	void enableKeepWholeHistory(bool enable = true)
	{
		m_keep_whole_history = enable;
	}
	bool isEnabledKeepWholeHistory() const { return m_keep_whole_history; }

	/** Dump all stats to a Comma Separated Values (CSV) file. \sa dumpAllStats
	 */
	void saveToCSVFile(const std::string& csv_file) const;
	/** Dump all stats to a Matlab/Octave (.m) file. \sa dumpAllStats */
	void saveToMFile(const std::string& m_file) const;
	void registerUserMeasure(
		const std::string_view& event_name, const double value,
		const bool is_time = false) noexcept;

	const std::string& getName() const noexcept { return m_name; }
	void setName(const std::string& name) noexcept { m_name = name; }

	/** Start of a named section \sa enter */
	inline void enter(const std::string_view& func_name) noexcept
	{
		if (m_enabled) do_enter(func_name);
	}
	/** End of a named section \return The ellapsed time, in seconds or 0 if
	 * disabled. \sa enter */
	inline double leave(const std::string_view& func_name) noexcept
	{
		return m_enabled ? do_leave(func_name) : 0;
	}
	/** Return the mean execution time of the given "section", or 0 if it hasn't
	 * ever been called "enter" with that section name */
	double getMeanTime(const std::string& name) const;
	/** Return the last execution time of the given "section", or 0 if it hasn't
	 * ever been called "enter" with that section name */
	double getLastTime(const std::string& name) const;
};  // End of class def.

/** A safe way to call enter() and leave() of a mrpt::system::CTimeLogger upon
 * construction and destruction of
 * this auxiliary object, making sure that leave() will be called upon
 * exceptions, etc.
 * Usage mode #1 (scoped):
 * \code
 *    CTimeLogger logger;
 *    // ...
 *    { // Start of scope to be monitorized
 *       CTimeLoggerEntry tle(logger,"operation-name");
 *
 *       // do whatever
 *
 *    } // End of scope
 *    // **DO NOT** call tle.stop() explicitly here, it's called by its dtor
 * \endcode
 *
 * Usage mode #2 (unscoped):
 * \code
 *    CTimeLogger logger;
 *    // ...
 *
 *    CTimeLoggerEntry tle(logger,"operation-name");
 *    // do whatever
 *    tle.stop();
 *
 *    // tle dtor does nothing else, since you already called stop()
 * \endcode
 *
 * \ingroup mrpt_system_grp
 */
struct CTimeLoggerEntry
{
	CTimeLoggerEntry(
		const CTimeLogger& logger, const std::string_view& section_name);
	~CTimeLoggerEntry();
	CTimeLogger& m_logger;
	void stop();  //!< for correct use, see docs for CTimeLoggerEntry

   private:
	// Note we cannot store the string_view since we have no guarantees of the
	// life-time of the provided string buffer.
	const std::string m_section_name;
	mrpt::Clock::time_point m_entry;
	bool stopped_{false};
};

/** A helper class to save CSV stats upon self destruction, for example, at the
 * end of a program run. The target file will be named after timelogger's name.
 * \ingroup mrpt_system_grp
 */
struct CTimeLoggerSaveAtDtor
{
	mrpt::system::CTimeLogger& m_tm;

	CTimeLoggerSaveAtDtor(mrpt::system::CTimeLogger& tm) : m_tm(tm) {}
	~CTimeLoggerSaveAtDtor();
};

/** @name Auxiliary stuff for the global profiler used in MRPT_START / MRPT_END
  macros.
  @{ */
void global_profiler_enter(const char* func_name) noexcept;
void global_profiler_leave(const char* func_name) noexcept;
mrpt::system::CTimeLogger& global_profiler_getref() noexcept;
/** @} */

}  // namespace mrpt::system
