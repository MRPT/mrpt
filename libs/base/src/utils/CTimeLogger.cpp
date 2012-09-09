/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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

void CTimeLogger::getStats(std::map<std::string,TCallStats> &out_stats) const
{
	out_stats.clear();
	for (map<string,TCallData>::const_iterator i=m_data.begin();i!=m_data.end();++i)
	{
		TCallStats &cs = out_stats[i->first];
		cs.min_t   = i->second.min_t;
		cs.max_t   = i->second.max_t;
		cs.total_t = i->second.mean_t;
		cs.mean_t  = i->second.n_calls ? i->second.mean_t/i->second.n_calls : 0;
		cs.n_calls = i->second.n_calls;
	}
}

std::string CTimeLogger::getStatsAsText(const size_t column_width)  const
{
	std::string s;

	s+="--------------------------- MRPT CTimeLogger report --------------------------\n";
	s+="           FUNCTION                         #CALLS  MIN.T  MEAN.T MAX.T TOTAL \n";
	s+="------------------------------------------------------------------------------\n";
	for (map<string,TCallData>::const_iterator i=m_data.begin();i!=m_data.end();++i)
	{
		const string sMinT   = unitsFormat(i->second.min_t,1,false);
		const string sMaxT   = unitsFormat(i->second.max_t,1,false);
		const string sTotalT = unitsFormat(i->second.mean_t,1,false);
		const string sMeanT  = unitsFormat(i->second.n_calls ? i->second.mean_t/i->second.n_calls : 0,1,false);

		s+=format("%s %7u %6ss %6ss %6ss %6ss\n",
			aux_format_string_multilines(i->first,40).c_str(),
			static_cast<unsigned int>(i->second.n_calls),
			sMinT.c_str(),
			sMeanT.c_str(),
			sMaxT.c_str(),
			sTotalT.c_str() );
	}

	s+="---------------------- End of MRPT CTimeLogger report ------------------------\n";

	return s;
}

void CTimeLogger::saveToCSVFile(const std::string &csv_file)  const
{
	std::string s;
	s+="FUNCTION, #CALLS, MIN.T, MEAN.T, MAX.T, TOTAL.T\n";
	for (map<string,TCallData>::const_iterator i=m_data.begin();i!=m_data.end();++i)
	{
		s+=format("\"%s\",\"%7u\",\"%e\",\"%e\",\"%e\",\"%e\"\n",
			i->first.c_str(),
			static_cast<unsigned int>(i->second.n_calls),
			i->second.min_t,
			i->second.n_calls ? i->second.mean_t/i->second.n_calls : 0,
			i->second.max_t,
			i->second.mean_t );
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
