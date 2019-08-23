/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "system-precomp.h"  // Precompiled headers

#include <mrpt/core/bits_math.h>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/string_utils.h>
#include <algorithm>
#include <fstream>
#include <iostream>

using namespace mrpt;
using namespace mrpt::system;
using namespace std;

struct MyGlobalProfiler : public mrpt::system::CTimeLogger
{
	MyGlobalProfiler() : mrpt::system::CTimeLogger("MRPT_global_profiler") {}
	~MyGlobalProfiler() override
	{
		try
		{
			if (!m_data.empty())
			{
				const std::string sFil("mrpt-global-profiler.csv");
				this->saveToCSVFile(sFil);
				std::cout << "[MRPT global profiler] Write stats to: " << sFil
						  << std::endl;
			}
		}
		catch (...)
		{
		}
	}
	MyGlobalProfiler(const MyGlobalProfiler&) = delete;
	MyGlobalProfiler(MyGlobalProfiler&&) = delete;
	MyGlobalProfiler& operator=(const MyGlobalProfiler&) = delete;
	MyGlobalProfiler& operator=(MyGlobalProfiler&&) = delete;
};
static MyGlobalProfiler global_profiler;

namespace mrpt::system
{
CTimeLogger& global_profiler_getref() noexcept { return global_profiler; }
void global_profiler_enter(const char* func_name) noexcept
{
	global_profiler.enter(func_name);
}
void global_profiler_leave(const char* func_name) noexcept
{
	global_profiler.leave(func_name);
}
}  // namespace mrpt::system

CTimeLogger::CTimeLogger(
	bool enabled, const std::string& name, const bool keep_whole_history)
	: COutputLogger("CTimeLogger"),
	  m_tictac(),
	  m_enabled(enabled),
	  m_name(name),
	  m_keep_whole_history(keep_whole_history)
{
	m_tictac.Tic();
}

CTimeLogger::~CTimeLogger()
{
	// Dump all stats:
	if (!m_data.empty())  // If logging is disabled, do nothing...
		dumpAllStats();
}

void CTimeLogger::clear(bool deep_clear)
{
	if (deep_clear)
		m_data.clear();
	else
	{
		for (auto& e : m_data) e.second = TCallData();
	}
}

std::string aux_format_string_multilines(const std::string& s, const size_t len)
{
	std::string ret;

	for (size_t p = 0; p < s.size(); p += len)
	{
		ret += rightPad(s.substr(p), len, true);
		if (p + len < s.size()) ret += "\n";
	}
	return ret;
}

void CTimeLogger::getStats(std::map<std::string, TCallStats>& out_stats) const
{
	out_stats.clear();
	for (const auto& e : m_data)
	{
		TCallStats& cs = out_stats[std::string(e.first)];
		cs.min_t = e.second.min_t;
		cs.max_t = e.second.max_t;
		cs.total_t = e.second.mean_t;
		cs.mean_t = e.second.n_calls ? e.second.mean_t / e.second.n_calls : 0;
		cs.n_calls = e.second.n_calls;
		cs.last_t = e.second.last_t;
	}
}

std::string CTimeLogger::getStatsAsText(const size_t column_width) const
{
	using std::string;
	using namespace std::string_literals;

	string stats_text;
	string name_tmp = m_name.size() != 0 ? " "s + m_name + ": "s : " "s;
	string mrpt_string = "MRPT CTimeLogger report "s;

	string top_header(name_tmp + mrpt_string);
	// append dashes to the header to reach column_width
	{
		const auto space_to_fill = top_header.size() < column_width
									   ? (column_width - top_header.size()) / 2
									   : 2;
		std::string dashes_half(space_to_fill, '-');
		top_header = dashes_half + top_header + dashes_half;
		if (dashes_half.size() % 2)
		{  // what if column_width-top_header.size() is odd?
			top_header += '-';
		}
	}

	std::string middle_header(
		"           FUNCTION                         #CALLS  MIN.T  MEAN.T "
		"MAX.T TOTAL ");
	std::string bottom_header(column_width, '-');

	stats_text += top_header + "\n"s;
	stats_text += middle_header + "\n"s;
	stats_text += bottom_header + "\n"s;

	// for all the timed sections: sort by inserting into a std::map
	using NameAndCallData = std::map<std::string_view, TCallData>;
	NameAndCallData stat_strs;
	for (const auto& i : m_data) stat_strs[i.first] = i.second;

	// format tree-like patterns like:
	//  ----------
	//  foobar
	//  foobar.a
	//  foobar.b
	//  ----------
	//  like:
	//  ----------
	//  foobar
	//  +-> a
	//  +-> b
	//  ----------
	std::string last_parent;
	for (const auto& i : stat_strs)
	{
		string line = string(i.first);  // make a copy

		const auto dot_pos = line.find(".");
		if (dot_pos == std::string::npos)
		{
			last_parent = line;
		}
		else
		{
			const auto parent_pos = line.find(last_parent);
			if (parent_pos != std::string::npos)
			{
				line = "+-> "s + line.substr(dot_pos);
			}
		}

		const string sMinT = unitsFormat(i.second.min_t, 1, false);
		const string sMaxT = unitsFormat(i.second.max_t, 1, false);
		const string sTotalT = unitsFormat(i.second.mean_t, 1, false);
		const string sMeanT = unitsFormat(
			i.second.n_calls ? i.second.mean_t / i.second.n_calls : 0, 1,
			false);

		stats_text += mrpt::format(
			"%s %7u %6s%c %6s%c %6s%c %6s%c\n",
			aux_format_string_multilines(line, 39).c_str(),
			static_cast<unsigned int>(i.second.n_calls), sMinT.c_str(),
			i.second.has_time_units ? 's' : ' ', sMeanT.c_str(),
			i.second.has_time_units ? 's' : ' ', sMaxT.c_str(),
			i.second.has_time_units ? 's' : ' ', sTotalT.c_str(),
			i.second.has_time_units ? 's' : ' ');
	}

	std::string footer(top_header);
	stats_text += footer + "\n";

	return stats_text;
}

void CTimeLogger::saveToCSVFile(const std::string& csv_file) const
{
	std::string s;
	s += "FUNCTION, #CALLS, LAST.T, MIN.T, MEAN.T, MAX.T, TOTAL.T [, "
		 "WHOLE_HISTORY]\n";
	for (const auto& i : m_data)
	{
		s += format(
			"\"%.*s\",%7u,%e,%e,%e,%e,%e", static_cast<int>(i.first.size()),
			i.first.data(), static_cast<unsigned int>(i.second.n_calls),
			i.second.last_t, i.second.min_t,
			i.second.n_calls ? i.second.mean_t / i.second.n_calls : 0,
			i.second.max_t, i.second.mean_t);

		if (i.second.whole_history)
		{
			const auto& wh = i.second.whole_history.value();
			for (const double v : wh)
			{
				s += ", ";
				s += std::to_string(v);
			}
		}
		s += "\n";
	}
	std::ofstream(csv_file) << s;
}

void CTimeLogger::saveToMFile(const std::string& file) const
{
	using std::string;
	using namespace std::string_literals;

	string s;
	s += "function [s] = "s + mrpt::system::extractFileName(file) +
		 "()\n"
		 "s = struct();\n"
		 "s.whole = struct();\n\n"s;

	std::string s_names = "s.names={"s;
	std::string s_counts = "s.count=["s;
	std::string s_mins = "s.min=["s;
	std::string s_maxs = "s.max=["s;
	std::string s_means = "s.mean=["s;

	for (const auto& i : m_data)
	{
		s_names += "'"s + i.first + "',"s;
		s_counts += std::to_string(i.second.n_calls) + ","s;
		s_mins += mrpt::format("%e,", i.second.min_t);
		s_maxs += mrpt::format("%e,", i.second.max_t);
		s_means += mrpt::format("%e,", i.second.mean_t);

		if (i.second.whole_history)
		{
			string clean_name =
				mrpt::system::fileNameStripInvalidChars(i.first);
			std::replace(clean_name.begin(), clean_name.end(), '.', '_');
			std::replace(clean_name.begin(), clean_name.end(), '-', '_');
			std::replace(clean_name.begin(), clean_name.end(), '+', '_');
			std::replace(clean_name.begin(), clean_name.end(), '*', '_');

			s += "s.whole."s + clean_name + "=[";
			const auto& wh = i.second.whole_history.value();
			for (const double v : wh) s += mrpt::format("%e,", v);
			s += "];\n";
		}
	}
	s_names += "};\n"s;
	s_counts += "];\n"s;
	s_mins += "];\n"s;
	s_maxs += "];\n"s;
	s_means += "];\n"s;

	s += s_names;
	s += s_counts;
	s += s_mins;
	s += s_maxs;
	s += s_means;

	s += "\n"s;
	std::ofstream(file) << s;
}

void CTimeLogger::dumpAllStats(const size_t column_width) const
{
	MRPT_LOG_INFO_STREAM("dumpAllStats:\n" << getStatsAsText(column_width));
}

void CTimeLogger::do_enter(const std::string_view& func_name) noexcept
{
	TCallData* d_ptr = m_data.find_or_alloc(std::string(func_name));
	if (!d_ptr)
	{
		std::cerr << "[CTimeLogger::do_enter] Warning: skipping due to hash "
					 "collision.\n";
		return;
	}
	auto& d = *d_ptr;
	d.n_calls++;
	d.open_calls.push(0);  // Dummy value, it'll be written below
	d.open_calls.top() = m_tictac.Tac();  // to avoid possible delays.
}

double CTimeLogger::do_leave(const std::string_view& func_name) noexcept
{
	const double tim = m_tictac.Tac();

	TCallData* d_ptr = m_data.find_or_alloc(std::string(func_name));
	if (!d_ptr)
	{
		std::cerr << "[CTimeLogger::do_enter] Warning: skipping due to hash "
					 "collision.\n";
		return .0;
	}
	auto& d = *d_ptr;

	if (!d.open_calls.empty())
	{
		const double At = tim - d.open_calls.top();
		d.open_calls.pop();

		d.last_t = At;
		d.mean_t += At;
		if (d.n_calls == 1)
		{
			d.min_t = At;
			d.max_t = At;
		}
		else
		{
			mrpt::keep_min(d.min_t, At);
			mrpt::keep_max(d.max_t, At);
		}
		if (m_keep_whole_history)
		{
			// Init the first time:
			if (!d.whole_history)
				d.whole_history = decltype(d.whole_history)::value_type();
			// Append to history:
			d.whole_history.value().push_back(At);
		}
		return At;
	}
	else
		return 0;  // This shouldn't happen!
}

void CTimeLogger::registerUserMeasure(
	const std::string_view& event_name, const double value,
	const bool is_time) noexcept
{
	if (!m_enabled) return;
	TCallData* d_ptr = m_data.find_or_alloc(std::string(event_name));
	if (!d_ptr)
	{
		std::cerr << "[CTimeLogger::do_enter] Warning: skipping due to hash "
					 "collision.\n";
		return;
	}
	auto& d = *d_ptr;

	d.has_time_units = is_time;
	d.last_t = value;
	d.mean_t += value;
	if (++d.n_calls == 1)
	{
		d.min_t = value;
		d.max_t = value;
	}
	else
	{
		mrpt::keep_min(d.min_t, value);
		mrpt::keep_max(d.max_t, value);
	}
	if (m_keep_whole_history)
	{
		// Init the first time:
		if (!d.whole_history)
			d.whole_history = decltype(d.whole_history)::value_type();
		// Append to history:
		d.whole_history.value().push_back(value);
	}
}

CTimeLogger::TCallData::TCallData() = default;

double CTimeLogger::getMeanTime(const std::string& name) const
{
	TDataMap::const_iterator it = m_data.find(name);
	if (it == m_data.end())
		return 0;
	else
		return it->second.n_calls ? it->second.mean_t / it->second.n_calls : 0;
}
double CTimeLogger::getLastTime(const std::string& name) const
{
	TDataMap::const_iterator it = m_data.find(name);
	if (it == m_data.end())
		return 0;
	else
		return it->second.last_t;
}

CTimeLoggerEntry::CTimeLoggerEntry(
	const CTimeLogger& logger, const std::string_view& section_name)
	: m_logger(const_cast<CTimeLogger&>(logger)), m_section_name(section_name)
{
	m_entry = mrpt::Clock::now();
}
void CTimeLoggerEntry::stop()
{
	if (stopped_) return;
	const auto leave = mrpt::Clock::now();
	const double dt = mrpt::system::timeDifference(m_entry, leave);

	m_logger.registerUserMeasure(m_section_name, dt, true);
	stopped_ = true;
}

CTimeLoggerEntry::~CTimeLoggerEntry()
{
	try
	{
		stop();
	}
	catch (const std::exception& e)
	{
		std::cerr << "[~CTimeLoggerEntry] Exception:\n"
				  << mrpt::exception_to_str(e);
	}
}

CTimeLoggerSaveAtDtor::~CTimeLoggerSaveAtDtor()
{
	try
	{
		using namespace std::string_literals;
		auto name = m_tm.getName() + ".m"s;
		name = fileNameStripInvalidChars(name);

		m_tm.logStr(
			LVL_INFO,
			"[CTimeLoggerSaveAtDtor] Saving stats to: `"s + name + "`"s);
		m_tm.saveToMFile(name);
	}
	catch (const std::exception& e)
	{
		std::cerr << "[~CTimeLoggerSaveAtDtor] Exception:\n"
				  << mrpt::exception_to_str(e);
	}
}
