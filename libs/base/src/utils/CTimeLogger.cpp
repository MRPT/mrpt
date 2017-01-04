/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/utils/CTimeLogger.h>
#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/system/string_utils.h>

#include <iostream>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace std;

CTimeLogger mrpt::utils::global_profiler;

namespace mrpt
{
	namespace utils
	{
		void global_profiler_enter(const char *func_name) MRPT_NO_THROWS {
			global_profiler.enter(func_name);
		}
		void global_profiler_leave(const char *func_name) MRPT_NO_THROWS {
			global_profiler.leave(func_name);
		}
	}
}

CTimeLogger::CTimeLogger(bool enabled/*=true*/, const std::string& name/*=""*/) :
	COutputLogger("CTimeLogger"),
	m_tictac(),
	m_enabled(enabled),
	m_name(name)
{
	m_tictac.Tic();
}

CTimeLogger::~CTimeLogger()
{
	// Dump all stats:
    if (!m_data.empty()) // If logging is disabled, do nothing...
        dumpAllStats();
}

CTimeLogger::CTimeLogger(const CTimeLogger&o) :
	COutputLogger(o),
	m_enabled(o.m_enabled),
	m_name(o.m_name),
	m_data(o.m_data)
{
}
CTimeLogger &CTimeLogger::operator =(const CTimeLogger&o)
{
	COutputLogger::operator=(o);
	m_enabled = o.m_enabled;
	m_name = o.m_name;
	m_data = o.m_data;
	return *this;
}
#if MRPT_HAS_CXX11
CTimeLogger::CTimeLogger(CTimeLogger&&o) :
	COutputLogger(o),
	m_enabled(o.m_enabled),
	m_name(o.m_name),
	m_data(o.m_data)
{
}
CTimeLogger &CTimeLogger::operator =(CTimeLogger&&o)
{
	COutputLogger::operator=(o);
	m_enabled = o.m_enabled;
	m_name = o.m_name;
	m_data = o.m_data;
	return *this;
}
#endif


void CTimeLogger::clear(bool deep_clear)
{
	if (deep_clear)
		m_data.clear();
	else
	{
		for (auto &e : m_data)
			e.second = TCallData();
	}
}

std::string  aux_format_string_multilines(const std::string &s, const size_t len)
{
	std::string ret;

	for (size_t p=0;p<s.size();p+=len)
	{
		ret+=rightPad(s.c_str()+p,len,true);
		if (p+len<s.size())
			ret+="\n";
	}
	return ret;
}

void CTimeLogger::getStats(std::map<std::string,TCallStats> &out_stats) const
{
	out_stats.clear();
	for (const auto e : m_data)
	{
		TCallStats &cs = out_stats[e.first];
		cs.min_t   = e.second.min_t;
		cs.max_t   = e.second.max_t;
		cs.total_t = e.second.mean_t;
		cs.mean_t  = e.second.n_calls ? e.second.mean_t/ e.second.n_calls : 0;
		cs.n_calls = e.second.n_calls;
		cs.last_t  = e.second.last_t;
	}
}

std::string CTimeLogger::getStatsAsText(const size_t column_width)  const
{
	std::string stats_text;
	std::string name_tmp = m_name.size() ? " " + m_name + ": " : " ";
	std::string mrpt_string = "MRPT CTimeLogger report ";

	std::string top_header(name_tmp + mrpt_string);
	// append dashes to the header to reach column_width
	{
		int space_to_fill = top_header.size() < column_width?
			(column_width-top_header.size())/2 : 2;
		std::string dashes_half(space_to_fill, '-');
		top_header = dashes_half + top_header + dashes_half;
		if (dashes_half.size() % 2)	{ // what if column_width-top_header.size() is odd?
			top_header += '-';
		}
	}

	std::string middle_header("           FUNCTION                         #CALLS  MIN.T  MEAN.T MAX.T TOTAL ");
	std::string bottom_header(column_width, '-');

	stats_text += top_header + "\n";
	stats_text += middle_header + "\n";
	stats_text += bottom_header + "\n";

	// for all the timed sections
	for (const auto i : m_data)
	{
		const string sMinT   = unitsFormat(i.second.min_t,1,false);
		const string sMaxT   = unitsFormat(i.second.max_t,1,false);
		const string sTotalT = unitsFormat(i.second.mean_t,1,false);
		const string sMeanT  = unitsFormat(i.second.n_calls ? i.second.mean_t/i.second.n_calls : 0,1,false);

		stats_text+=format("%s %7u %6s%c %6s%c %6s%c %6s%c\n",
			aux_format_string_multilines(i.first,39).c_str(),
			static_cast<unsigned int>(i.second.n_calls),
			sMinT.c_str(), i.second.has_time_units ? 's':' ',
			sMeanT.c_str(),i.second.has_time_units ? 's':' ',
			sMaxT.c_str(),i.second.has_time_units ? 's':' ',
			sTotalT.c_str(),i.second.has_time_units ? 's':' ' );
	}

	std::string footer(top_header);
	stats_text += footer + "\n";

	return stats_text;
}

void CTimeLogger::saveToCSVFile(const std::string &csv_file)  const
{
	std::string s;
	s+="FUNCTION, #CALLS, LAST.T, MIN.T, MEAN.T, MAX.T, TOTAL.T\n";
	for (const auto &i : m_data)
	{
		s+=format("\"%s\",\"%7u\",\"%e\",\"%e\",\"%e\",\"%e\",\"%e\"\n",
			i.first.c_str(),
			static_cast<unsigned int>(i.second.n_calls),
			i.second.last_t,
			i.second.min_t,
			i.second.n_calls ? i.second.mean_t/i.second.n_calls : 0,
			i.second.max_t,
			i.second.mean_t );
	}
	CFileOutputStream(csv_file).printf("%s",s.c_str() );
}

void CTimeLogger::dumpAllStats(const size_t  column_width) const
{
	MRPT_LOG_INFO_STREAM << "dumpAllStats:\n" << getStatsAsText(column_width);
}

void CTimeLogger::do_enter(const char *func_name)
{
	const string  s = func_name;
	TCallData &d = m_data[s];

	d.n_calls++;
	d.open_calls.push(0);  // Dummy value, it'll be written below
	d.open_calls.top() = m_tictac.Tac(); // to avoid possible delays.
}

double CTimeLogger::do_leave(const char *func_name)
{
	const double tim = m_tictac.Tac();

	const string  s = func_name;
	TCallData &d = m_data[s];

	if (!d.open_calls.empty())
	{
		const double At = tim - d.open_calls.top();
		d.open_calls.pop();

		d.last_t = At;
		d.mean_t+=At;
		if (d.n_calls==1)
		{
			d.min_t= At;
			d.max_t= At;
		}
		else
		{
			mrpt::utils::keep_min( d.min_t, At);
			mrpt::utils::keep_max( d.max_t, At);
		}
		return At;
	}
	else return 0; // This shouldn't happen!
}

void CTimeLogger::registerUserMeasure(const char *event_name, const double value)
{
    if (!m_enabled) return;
	const string  s = event_name;
	TCallData &d = m_data[s];

	d.has_time_units = false;
	d.last_t = value;
	d.mean_t+=value;
	if (++d.n_calls==1)
	{
		d.min_t= value;
		d.max_t= value;
	}
	else
	{
		mrpt::utils::keep_min( d.min_t, value);
		mrpt::utils::keep_max( d.max_t, value);
	}
}

CTimeLogger::TCallData::TCallData() :
	n_calls	(0),
	min_t	(0),
	max_t	(0),
	mean_t	(0),
	last_t	(0),
	has_time_units(true)
{
}

double CTimeLogger::getMeanTime(const std::string &name)  const
{
	TDataMap::const_iterator it = m_data.find(name);
	if (it==m_data.end())
		 return 0;
	else return it->second.n_calls ? it->second.mean_t/it->second.n_calls : 0;
}
double CTimeLogger::getLastTime(const std::string &name) const
{
	TDataMap::const_iterator it = m_data.find(name);
	if (it == m_data.end())
		return 0;
	else return it->second.last_t;
}

CTimeLoggerEntry::CTimeLoggerEntry(CTimeLogger &logger, const char*section_name ) : m_logger(logger),m_section_name(section_name)
{
	m_logger.enter(m_section_name);
}
CTimeLoggerEntry::~CTimeLoggerEntry()
{
	m_logger.leave(m_section_name);
}
