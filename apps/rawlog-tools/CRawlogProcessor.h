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

#ifndef RAWLOG_PROCESSOR_H
#define RAWLOG_PROCESSOR_H

#include <mrpt/slam/CRawlog.h>
#include <mrpt/base.h>

// Aparently, TCLAP headers can't be included in more than one source file
//  or duplicated linking symbols appear! -> Use forward declarations instead:
// #include <mrpt/otherlibs/tclap/CmdLine.h>
namespace TCLAP {
	class CmdLine;
}

namespace mrpt
{
	namespace rawlogtools
	{
		/** A virtual class that implements the common stuff around parsing a rawlog file
		  * and (optionally) display a progress indicator to the console.
		  */
		class CRawlogProcessor
		{
		protected:
			mrpt::utils::CFileGZInputStream	& m_in_rawlog;
			TCLAP::CmdLine			& m_cmdline;
			bool					verbose;
			mrpt::system::TTimeStamp m_last_console_update;
			mrpt::utils::CTicTac	m_timParse;

		public:
			uint64_t		m_filSize;
			size_t			m_rawlogEntry;
			double 			m_timToParse; // Public variable, at end will hold ellapsed time.

			// Ctor
			CRawlogProcessor(mrpt::utils::CFileGZInputStream &_in_rawlog, TCLAP::CmdLine &_cmdline, bool _verbose) :
				m_in_rawlog(_in_rawlog),m_cmdline(_cmdline), verbose(_verbose), m_last_console_update( mrpt::system::now() ), m_rawlogEntry(0)
			{
				m_filSize = _in_rawlog.getTotalBytesCount();
			}


			// The main method:
			void doProcessRawlog()
			{
				// The 3 different objects we can read from a rawlog:
				mrpt::slam::CActionCollectionPtr actions;
				mrpt::slam::CSensoryFramePtr     SF;
				mrpt::slam::CObservationPtr      obs;

				m_timParse.Tic();

				// Parse the entire rawlog:
				while (mrpt::slam::CRawlog::getActionObservationPairOrObservation(
					m_in_rawlog,
					actions,SF, obs,
					m_rawlogEntry ) )
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
					if ( mrpt::system::timeDifference(m_last_console_update,tNow)>0.25)
					{
						m_last_console_update = tNow;
						uint64_t fil_pos = m_in_rawlog.getPosition();
						if(verbose)
						{
							std::cout << mrpt::format("Progress: %7u objects --- Pos: %9sB/%c%9sB \r",
							(unsigned int)m_rawlogEntry,
							mrpt::system::unitsFormat(fil_pos).c_str(),
							(fil_pos>m_filSize ? '>':' '),
							mrpt::system::unitsFormat(m_filSize).c_str()
							);  // \r -> don't go to the next line...

							std::cout.flush();
						}
					}

					// Do whatever:
					processOneEntry(actions,SF,obs);

					// Post process:
					OnPostProcess(actions,SF,obs);

					// Clear read objects:
					actions.clear_unique();
					SF.clear_unique();
					obs.clear_unique();
				}; // end while

				if(verbose) std::cout << "\n"; // new line after the "\r".

				m_timToParse = m_timParse.Tac();

			} // end doProcessRawlog


			// The virtual method of the user to be invoked for each read object:
			//  Return false to abort and stop the read loop.
			virtual bool processOneEntry(
				mrpt::slam::CActionCollectionPtr &actions,
				mrpt::slam::CSensoryFramePtr     &SF,
				mrpt::slam::CObservationPtr      &obs) = 0;

			// This method can be reimplemented to save the modified object to an output stream.
			virtual void OnPostProcess(
				mrpt::slam::CActionCollectionPtr &actions,
				mrpt::slam::CSensoryFramePtr     &SF,
				mrpt::slam::CObservationPtr      &obs)
			{
				// Default: Do nothing
			}

		}; // end CRawlogProcessor

		/** A virtual class that implements the common stuff around parsing a rawlog file
		  * and (optionally) display a progress indicator to the console.
		  */
		class CRawlogProcessorOnEachObservation : public CRawlogProcessor
		{
		public:
			CRawlogProcessorOnEachObservation(mrpt::utils::CFileGZInputStream &in_rawlog, TCLAP::CmdLine &cmdline, bool verbose) :
				CRawlogProcessor(in_rawlog,cmdline,verbose)
			{
			}

			virtual bool processOneEntry(
				mrpt::slam::CActionCollectionPtr &actions,
				mrpt::slam::CSensoryFramePtr     &SF,
				mrpt::slam::CObservationPtr      &obs)
			{
				// Process each observation individually, either from "obs" or each within a "SF":
				for (size_t idxObs=0; true; idxObs++)
				{
					mrpt::slam::CObservationPtr  obs_indiv;
					if (obs)
					{
						if (idxObs>0)  break;
						obs_indiv = obs;
					}
					else if (SF)
					{
						if (idxObs>=SF->size()) break;
						obs_indiv = SF->getObservationByIndex(idxObs);
					}
					else break; // shouldn't...

					// Process "obs_indiv":
					ASSERT_(obs_indiv)
					if (!processOneObservation(obs_indiv))
						return false;
				}

				return true; // No error.
			}

			// To be implemented by the user. Return false on any error to abort processing.
			virtual bool processOneObservation(mrpt::slam::CObservationPtr  &obs) = 0;


		}; // end CRawlogProcessorOnEachObservation



		/** A specialization of CRawlogProcessorOnEachObservation that handles the common case of
		  *  filtering entries in a rawlog depending on the return value of a user function.
		  */
		class CRawlogProcessorFilterObservations : public CRawlogProcessorOnEachObservation
		{
		public:
			mrpt::utils::CFileGZOutputStream 	&m_out_rawlog;
			size_t  				m_entries_removed, m_entries_parsed;

			CRawlogProcessorFilterObservations(mrpt::utils::CFileGZInputStream &in_rawlog, TCLAP::CmdLine &cmdline, bool verbose, mrpt::utils::CFileGZOutputStream &out_rawlog) :
				CRawlogProcessorOnEachObservation(in_rawlog,cmdline,verbose),
				m_out_rawlog(out_rawlog),
				m_entries_removed(0),
				m_entries_parsed(0)
			{
			}

			/** To be implemented by users: return false means the observation is  */
			virtual bool tellIfThisObsPasses(mrpt::slam::CObservationPtr  &obs) = 0;

			// Process each entry. Return false on any error to abort processing.
			virtual bool processOneObservation(mrpt::slam::CObservationPtr  &obs)
			{
				if (!tellIfThisObsPasses(obs))
				{
					obs.clear(); // Free object (all aliases)
					m_entries_removed++;
				}
				m_entries_parsed++;
				return true;
			}
			// Save those entries which are not NULL.
			virtual void OnPostProcess(
				mrpt::slam::CActionCollectionPtr &actions,
				mrpt::slam::CSensoryFramePtr     &SF,
				mrpt::slam::CObservationPtr      &obs)
			{
				if (actions)
				{
					ASSERT_(actions && SF)
					// Remove from SF those observations freed:
					mrpt::slam::CSensoryFrame::iterator it = SF->begin();
					while (it!=SF->end())
					{
						if ( (*it).present() )
							it++;
						else it = SF->erase(it);
					}
					// Save:
					m_out_rawlog << actions << SF;
				}
				else
				{
					if (obs)
						m_out_rawlog << obs;
				}
			}


		}; // end CRawlogProcessorOnEachObservation


	} // end NS
} // end NS

#endif

