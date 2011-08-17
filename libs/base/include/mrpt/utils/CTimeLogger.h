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
#ifndef  CTimeLogger_H
#define  CTimeLogger_H

#include <mrpt/utils/CTicTac.h>
#include <mrpt/utils/CDebugOutputCapable.h>

#include <stack>

namespace mrpt
{
	namespace utils
	{
		using namespace std;

		/** A versatile "profiler" that logs the time spent within each pair of calls to enter(X)-leave(X), among other stats.
		 *  The results can be dumped to cout or to Visual Studio's output panel.
		 *  Recursive methods are supported with no problems, that is, calling "enter(X) enter(X) ... leave(X) leave(X)".
		 * \note The default behavior is dumping all the information at destruction.
		 * \ingroup mrpt_base_grp
		 */
		class BASE_IMPEXP CTimeLogger : public mrpt::utils::CDebugOutputCapable
		{
		private:
			CTicTac		m_tictac;
			bool		m_enabled;

			//! Data of all the calls:
			struct TCallData
			{
				TCallData();

				size_t n_calls;
				double min_t,max_t,mean_t;
				stack<double,vector<double> >   open_calls;
			};

			map<string,TCallData>  m_data;

			void do_enter( const char *func_name );
			double do_leave( const char *func_name );

		public:
			CTimeLogger(bool enabled = true); //! Default constructor
			virtual ~CTimeLogger(); //!< Destructor
			std::string getStatsAsText(const size_t column_width=80) const; //!< Dump all stats to a multi-line text string. \sa dumpAllStats, saveToCVSFile
			void dumpAllStats(const size_t column_width=80) const; //!< Dump all stats through the CDebugOutputCapable interface. \sa getStatsAsText, saveToCVSFile
			void clear();
			void enable(bool enabled = true) { m_enabled = enabled; }
			void disable() { m_enabled = false; }
			void saveToCSVFile(const std::string &csv_file)  const; 	//!< Dump all stats to a Comma Separated Values (CSV) file. \sa dumpAllStats

			/** Start of a named section \sa enter */
			inline void enter( const char *func_name ) {
				if (m_enabled)
					do_enter(func_name);
			}
			/** End of a named section \return The ellapsed time, in seconds or 0 if disabled. \sa enter */
			inline double leave( const char *func_name ) {
				return m_enabled ? do_leave(func_name) : 0;
			}
			/** Return the mean execution time of the given "section", or 0 if it hasn't ever been called "enter" with that section name */
			double getMeanTime(const std::string &name) const;
		}; // End of class def.


		/** @name Auxiliary stuff for the global profiler used in MRPT_START / MRPT_END macros.
		  @{ */
		extern CTimeLogger BASE_IMPEXP global_profiler;
		void BASE_IMPEXP global_profiler_enter(const char *func_name) MRPT_NO_THROWS;
		void BASE_IMPEXP global_profiler_leave(const char *func_name) MRPT_NO_THROWS;
		/** @} */

	} // End of namespace
} // End of namespace
#endif
