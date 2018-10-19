/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#ifndef RAWLOG_PROCESSOR_H
#define RAWLOG_PROCESSOR_H

#include <mrpt/obs/CRawlog.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/system/os.h>
#include <mrpt/serialization/CArchive.h>

// Aparently, TCLAP headers can't be included in more than one source file
//  or duplicated linking symbols appear! -> Use forward declarations instead:
// #include <mrpt/otherlibs/tclap/CmdLine.h>
namespace TCLAP
{
class CmdLine;
}

namespace mrpt
{
namespace rawlogtools
{
/** A virtual class that implements the common stuff around parsing a rawlog
 * file
 * and (optionally) display a progress indicator to the console.
 */
class CRawlogProcessor
{
   protected:
	mrpt::io::CFileGZInputStream& m_in_rawlog;
	TCLAP::CmdLine& m_cmdline;
	bool verbose;
	mrpt::system::TTimeStamp m_last_console_update;
	mrpt::system::CTicTac m_timParse;

   public:
	uint64_t m_filSize;
	size_t m_rawlogEntry;
	double m_timToParse;  // Public variable, at end will hold ellapsed time.

	// Ctor
	CRawlogProcessor(
		mrpt::io::CFileGZInputStream& _in_rawlog, TCLAP::CmdLine& _cmdline,
		bool _verbose)
		: m_in_rawlog(_in_rawlog),
		  m_cmdline(_cmdline),
		  verbose(_verbose),
		  m_last_console_update(mrpt::system::now()),
		  m_rawlogEntry(0)
	{
		m_filSize = _in_rawlog.getTotalBytesCount();
	}

	// The main method:
	void doProcessRawlog()
	{
		// The 3 different objects we can read from a rawlog:
		mrpt::obs::CActionCollection::Ptr actions;
		mrpt::obs::CSensoryFrame::Ptr SF;
		mrpt::obs::CObservation::Ptr obs;

		m_timParse.Tic();

		// Parse the entire rawlog:
		auto arch = mrpt::serialization::archiveFrom(m_in_rawlog);
		while (mrpt::obs::CRawlog::getActionObservationPairOrObservation(
			arch, actions, SF, obs, m_rawlogEntry))
		{
			// Abort if the user presses ESC:
			if (mrpt::system::os::kbhit())
				if (27 == mrpt::system::os::getch())
				{
					std::cerr << "Aborted since user pressed ESC.\n";
					break;
				}

			// Update status to the console?
			const mrpt::system::TTimeStamp tNow = mrpt::system::now();
			if (mrpt::system::timeDifference(m_last_console_update, tNow) >
				0.25)
			{
				m_last_console_update = tNow;
				uint64_t fil_pos = m_in_rawlog.getPosition();
				if (verbose)
				{
					std::cout << mrpt::format(
						"Progress: %7u objects --- Pos: %9sB/%c%9sB \r",
						(unsigned int)m_rawlogEntry,
						mrpt::system::unitsFormat(fil_pos).c_str(),
						(fil_pos > m_filSize ? '>' : ' '),
						mrpt::system::unitsFormat(m_filSize)
							.c_str());  // \r -> don't go to the next line...

					std::cout.flush();
				}
			}

			// Do whatever:
			bool process_ret = processOneEntry(actions, SF, obs);

			// Post process:
			OnPostProcess(actions, SF, obs);

			// Clear read objects:
			actions.reset();
			SF.reset();
			obs.reset();

			if (!process_ret)
			{
				// Returning false means we should stop parsing the rest of the
				// rawlog:
				std::cerr << "\nParsing stopped due to request from Rawlog "
							 "filter implementation.\n";
				break;
			}
		};  // end while

		if (verbose) std::cout << "\n";  // new line after the "\r".

		m_timToParse = m_timParse.Tac();

	}  // end doProcessRawlog

	// The virtual method of the user to be invoked for each read object:
	//  Return false to abort and stop the read loop.
	virtual bool processOneEntry(
		mrpt::obs::CActionCollection::Ptr& actions,
		mrpt::obs::CSensoryFrame::Ptr& SF,
		mrpt::obs::CObservation::Ptr& obs) = 0;

	// This method can be reimplemented to save the modified object to an output
	// stream.
	virtual void OnPostProcess(
		mrpt::obs::CActionCollection::Ptr& actions,
		mrpt::obs::CSensoryFrame::Ptr& SF, mrpt::obs::CObservation::Ptr& obs)
	{
		MRPT_UNUSED_PARAM(actions);
		MRPT_UNUSED_PARAM(SF);
		MRPT_UNUSED_PARAM(obs);
		// Default: Do nothing
	}

};  // end CRawlogProcessor

/** A virtual class that implements the common stuff around parsing a rawlog
 * file
 * and (optionally) display a progress indicator to the console.
 */
class CRawlogProcessorOnEachObservation : public CRawlogProcessor
{
   public:
	CRawlogProcessorOnEachObservation(
		mrpt::io::CFileGZInputStream& in_rawlog, TCLAP::CmdLine& cmdline,
		bool enable_verbose)
		: CRawlogProcessor(in_rawlog, cmdline, enable_verbose)
	{
	}

	bool processOneEntry(
		mrpt::obs::CActionCollection::Ptr& actions,
		mrpt::obs::CSensoryFrame::Ptr& SF,
		mrpt::obs::CObservation::Ptr& obs) override
	{
		MRPT_UNUSED_PARAM(actions);
		// Process each observation individually, either from "obs" or each
		// within a "SF":
		for (size_t idxObs = 0; true; idxObs++)
		{
			mrpt::obs::CObservation::Ptr obs_indiv;
			if (obs)
			{
				if (idxObs > 0) break;
				obs_indiv = obs;
			}
			else if (SF)
			{
				if (idxObs >= SF->size()) break;
				obs_indiv = SF->getObservationByIndex(idxObs);
			}
			else
				break;  // shouldn't...

			// Process "obs_indiv":
			ASSERT_(obs_indiv);
			if (!processOneObservation(obs_indiv)) return false;
		}

		return true;  // No error.
	}

	// To be implemented by the user. Return false on any error to abort
	// processing.
	virtual bool processOneObservation(mrpt::obs::CObservation::Ptr& obs) = 0;

};  // end CRawlogProcessorOnEachObservation

/** A specialization of CRawlogProcessorOnEachObservation that handles the
 * common case of
 *  filtering entries in a rawlog depending on the return value of a user
 * function.
 */
class CRawlogProcessorFilterObservations
	: public CRawlogProcessorOnEachObservation
{
   public:
	mrpt::io::CFileGZOutputStream& m_out_rawlog;
	size_t m_entries_removed, m_entries_parsed;
	/** Set to true to indicate that we are sure we don't have to keep on
	 * reading. */
	bool m_we_are_done_with_this_rawlog;

	CRawlogProcessorFilterObservations(
		mrpt::io::CFileGZInputStream& in_rawlog, TCLAP::CmdLine& cmdline,
		bool enable_verbose, mrpt::io::CFileGZOutputStream& out_rawlog)
		: CRawlogProcessorOnEachObservation(in_rawlog, cmdline, enable_verbose),
		  m_out_rawlog(out_rawlog),
		  m_entries_removed(0),
		  m_entries_parsed(0),
		  m_we_are_done_with_this_rawlog(false)
	{
	}

	/** To be implemented by users: return false means the observation is  */
	virtual bool tellIfThisObsPasses(mrpt::obs::CObservation::Ptr& obs) = 0;

	// Process each entry. Return false on any error to abort processing.
	bool processOneObservation(mrpt::obs::CObservation::Ptr& obs) override
	{
		if (!tellIfThisObsPasses(obs))
		{
			obs.reset();  // Free object (all aliases)
			m_entries_removed++;
		}
		m_entries_parsed++;

		if (m_we_are_done_with_this_rawlog)
			return false;  // We are done, finish execution.

		return true;
	}
	// Save those entries which are not nullptr.
	void OnPostProcess(
		mrpt::obs::CActionCollection::Ptr& actions,
		mrpt::obs::CSensoryFrame::Ptr& SF,
		mrpt::obs::CObservation::Ptr& obs) override
	{
		if (actions)
		{
			ASSERT_(actions && SF);
			// Remove from SF those observations freed:
			auto it = SF->begin();
			while (it != SF->end())
			{
				if (*it)
					it++;
				else
					it = SF->erase(it);
			}
			// Save:
			mrpt::serialization::archiveFrom(m_out_rawlog) << actions << SF;
		}
		else
		{
			if (obs) mrpt::serialization::archiveFrom(m_out_rawlog) << obs;
		}
	}

};  // end CRawlogProcessorOnEachObservation

}  // namespace rawlogtools
}  // namespace mrpt

#endif
