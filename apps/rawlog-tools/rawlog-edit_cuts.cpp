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

#include "rawlog-edit-declarations.h"

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::system;
using namespace mrpt::rawlogtools;
using namespace std;


// ======================================================================
//		op_cut
// ======================================================================
DECLARE_OP_FUNCTION(op_cut)
{
	// A class to do this operation:
	class CRawlogProcessor_Cut : public CRawlogProcessorFilterObservations
	{
	protected:
		bool has_from_index, has_to_index, has_from_time, has_to_time;
		size_t m_from_index, m_to_index;
		double m_from_time,  m_to_time;


	public:
		CRawlogProcessor_Cut(
			mrpt::utils::CFileGZInputStream &in_rawlog, TCLAP::CmdLine &cmdline, bool verbose,
			CFileGZOutputStream &out_rawlog ) :
				CRawlogProcessorFilterObservations(in_rawlog,cmdline,verbose, out_rawlog),
				m_from_index(0), m_to_index(0),
				m_from_time(0),  m_to_time(0)
		{
			has_from_index = getArgValue<size_t>(cmdline,"from-index",m_from_index);
			has_to_index   = getArgValue<size_t>(cmdline,"to-index",  m_to_index);
			has_from_time  = getArgValue<double>(cmdline,"from-time",m_from_time);
			has_to_time    = getArgValue<double>(cmdline,"to-time",  m_to_time);

			if (!has_from_index && !has_to_index && !has_from_time && !has_to_time)
				throw std::runtime_error("cut: This operation needs at least one of: --from-index, --from-time, --to-index, --to-time.");

			if (verbose)
			{
				if (has_from_index) cout << "Using cut filter: from-index=" << m_from_index << endl;
				if (has_to_index)   cout << "Using cut filter:   to-index=" << m_to_index << endl;
				if (has_from_time)  cout << "Using cut filter: from-time =" << dateTimeLocalToString( time_tToTimestamp(m_from_time) ) << endl;
				if (has_to_time)    cout << "Using cut filter:   to-time =" << dateTimeLocalToString( time_tToTimestamp(m_to_time) ) << endl;
			}
		}

		/** To be implemented by users: return false means the observation is  */
		virtual bool tellIfThisObsPasses(mrpt::slam::CObservationPtr  &obs)
		{
			if (has_from_index && m_rawlogEntry<m_from_index)
					return false;

			if (has_from_time)
			{
				ASSERT_(obs->timestamp!=INVALID_TIMESTAMP);
				if (timestampToDouble(obs->timestamp)<m_from_time)
					return false;
			}
			if (has_to_index && m_rawlogEntry>m_to_index)
			{
				// TODO: We could also say to not read anymore...
				return false;
			}

			if (has_to_time)
			{
				ASSERT_(obs->timestamp!=INVALID_TIMESTAMP);
				if (timestampToDouble(obs->timestamp)>m_to_time)
					return false;
			}

			// All filters passed.
			return true;
		}
	};

	// Process
	// ---------------------------------
	TOutputRawlogCreator	outrawlog;
	CRawlogProcessor_Cut proc(in_rawlog,cmdline,verbose,outrawlog.out_rawlog);
	proc.doProcessRawlog();

	// Dump statistics:
	// ---------------------------------
	VERBOSE_COUT << "Time to process file (sec)        : " << proc.m_timToParse << "\n";
	VERBOSE_COUT << "Analyzed entries                  : " << proc.m_entries_parsed << "\n";
	VERBOSE_COUT << "Removed entries                   : " << proc.m_entries_removed << "\n";

}
