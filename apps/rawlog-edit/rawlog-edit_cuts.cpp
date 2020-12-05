/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "rawlog-edit-declarations.h"

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::rawlogtools;
using namespace std;
using namespace mrpt::io;

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
		double m_from_time, m_to_time;

	   public:
		CRawlogProcessor_Cut(
			mrpt::io::CFileGZInputStream& in_rawlog, TCLAP::CmdLine& cmdline,
			bool Verbose, mrpt::io::CFileGZOutputStream& out_rawlog)
			: CRawlogProcessorFilterObservations(
				  in_rawlog, cmdline, Verbose, out_rawlog),
			  m_from_index(0),
			  m_to_index(0),
			  m_from_time(0),
			  m_to_time(0)
		{
			has_from_index =
				getArgValue<size_t>(cmdline, "from-index", m_from_index);
			has_to_index = getArgValue<size_t>(cmdline, "to-index", m_to_index);
			has_from_time =
				getArgValue<double>(cmdline, "from-time", m_from_time);
			has_to_time = getArgValue<double>(cmdline, "to-time", m_to_time);

			if (!has_from_index && !has_to_index && !has_from_time &&
				!has_to_time)
				throw std::runtime_error(
					"cut: This operation needs at least one of: --from-index, "
					"--from-time, --to-index, --to-time.");

			if (verbose)
			{
				if (has_from_index)
					cout << "Using cut filter: from-index=" << m_from_index
						 << endl;
				if (has_to_index)
					cout << "Using cut filter:   to-index=" << m_to_index
						 << endl;
				if (has_from_time)
					cout << "Using cut filter: from-time ="
						 << dateTimeLocalToString(
								time_tToTimestamp(m_from_time))
						 << endl;
				if (has_to_time)
					cout << "Using cut filter:   to-time ="
						 << dateTimeLocalToString(time_tToTimestamp(m_to_time))
						 << endl;
			}
		}

		/** To be implemented by users: return false means entry must be removed
		 */
		bool tellIfThisActPasses(mrpt::obs::CAction::Ptr& act) override
		{
			ASSERT_(act);
			return tellIfThisOnePasses(act->timestamp);
		}

		/** To be implemented by users: return false means the observation is */
		bool tellIfThisObsPasses(mrpt::obs::CObservation::Ptr& obs) override
		{
			ASSERT_(obs);
			return tellIfThisOnePasses(obs->timestamp);
		}

		/** To be implemented by users: return false means the observation
		   is */
		bool tellIfThisOnePasses(mrpt::system::TTimeStamp t)
		{
			if (has_from_index && m_rawlogEntry < m_from_index) return false;

			if (has_from_time)
			{
				ASSERT_(t != INVALID_TIMESTAMP);
				if (timestampToDouble(t) < m_from_time) return false;
			}
			if (has_to_index && m_rawlogEntry > m_to_index)
			{
				// say not to read anymore...
				m_we_are_done_with_this_rawlog = true;
				return false;
			}

			if (has_to_time)
			{
				ASSERT_(t != INVALID_TIMESTAMP);
				if (timestampToDouble(t) > m_to_time)
				{
					// say not to read anymore...
					m_we_are_done_with_this_rawlog = true;
					return false;
				}
			}

			// All filters passed.
			return true;
		}
	};

	// Process
	// ---------------------------------
	TOutputRawlogCreator outrawlog;
	CRawlogProcessor_Cut proc(
		in_rawlog, cmdline, verbose, outrawlog.out_rawlog_io);
	proc.doProcessRawlog();

	// Dump statistics:
	// ---------------------------------
	VERBOSE_COUT << "Time to process file (sec)        : " << proc.m_timToParse
				 << "\n";
	VERBOSE_COUT << "Analyzed entries                  : "
				 << proc.m_entries_parsed << "\n";
	VERBOSE_COUT << "Removed entries                   : "
				 << proc.m_entries_removed << "\n";
}
