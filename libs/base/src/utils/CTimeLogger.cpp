/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/base.h>  // Precompiled headers

#include <mrpt/utils/CTimeLogger.h>
#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/system/string_utils.h>

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

CTimeLogger::CTimeLogger(bool enabled) : m_tictac(), m_enabled(enabled)
{
	m_tictac.Tic();
}

CTimeLogger::~CTimeLogger()
{
	// Dump all stats:
    if (!m_data.empty()) // If logging is disabled, do nothing...
        dumpAllStats();
}

void CTimeLogger::clear()
{
	m_data.clear();
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

std::string CTimeLogger::getStatsAsText(const size_t column_width)  const
{
	std::string s;

	s+="--------------------------- MRPT CTimeLogger report --------------------------\n";
	s+="           FUNCTION                                #CALLS  MIN.T  MEAN.T MAX.T\n";
	s+="------------------------------------------------------------------------------\n";
	for (map<string,TCallData>::const_iterator i=m_data.begin();i!=m_data.end();++i)
	{
		const string sMinT = unitsFormat(i->second.min_t,1,false);
		const string sMaxT = unitsFormat(i->second.max_t,1,false);
		const string sMeanT = unitsFormat(i->second.n_calls ? i->second.mean_t/i->second.n_calls : 0,1,false);
		s+=format("%s %7u %6ss %6ss %6ss\n",
			aux_format_string_multilines(i->first,47).c_str(),
			static_cast<unsigned int>(i->second.n_calls),
			sMinT.c_str(),
			sMeanT.c_str(),
			sMaxT.c_str() );
	}

	s+="---------------------- End of MRPT CTimeLogger report ------------------------\n";

	return s;
}

void CTimeLogger::saveToCSVFile(const std::string &csv_file)  const
{
	std::string s;
	s+="FUNCTION, #CALLS, MIN.T, MEAN.T, MAX.T\n";
	for (map<string,TCallData>::const_iterator i=m_data.begin();i!=m_data.end();++i)
	{
		s+=format("\"%s\",\"%7u\",\"%e\",\"%e\",\"%e\"\n",
			i->first.c_str(),
			static_cast<unsigned int>(i->second.n_calls),
			i->second.min_t,
			i->second.n_calls ? i->second.mean_t/i->second.n_calls : 0,
			i->second.max_t );
	}
	CFileOutputStream(csv_file).printf("%s",s.c_str() );
}

void CTimeLogger::dumpAllStats(const size_t  column_width) const
{
	string s = getStatsAsText(column_width);
	printf_debug("\n%s\n", s.c_str() );
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

CTimeLogger::TCallData::TCallData() :
	n_calls	(0),
	min_t	(0),
	max_t	(0),
	mean_t	(0)
{
}


double CTimeLogger::getMeanTime(const std::string &name)  const
{
	map<string,TCallData>::const_iterator it = m_data.find(name);
	if (it==m_data.end())
		 return 0;
	else return it->second.n_calls ? it->second.mean_t/it->second.n_calls : 0;
}
