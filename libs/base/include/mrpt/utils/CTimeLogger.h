/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  CTimeLogger_H
#define  CTimeLogger_H

#include <mrpt/utils/CTicTac.h>
#include <mrpt/utils/COutputLogger.h>
#include <mrpt/utils/compiler_fixes.h>
#include <mrpt/utils/mrpt_macros.h>
#include <mrpt/utils/ts_hash_map.h>
#include <vector>
#include <stack>
//#include <map>

namespace mrpt
{
	namespace utils
	{
		/** A versatile "profiler" that logs the time spent within each pair of calls to enter(X)-leave(X), among other stats.
		 *  The results can be dumped to cout or to Visual Studio's output panel.
		 *  Recursive methods are supported with no problems, that is, calling "enter(X) enter(X) ... leave(X) leave(X)".
		 *
		 *  This class can be also used to monitorize min/mean/max/total stats of any user-provided parameters via the method CTimeLogger::registerUserMeasure()
		 *
		 * \sa CTimeLoggerEntry
		 *
		 * \note The default behavior is dumping all the information at destruction.
		 * \ingroup mrpt_base_grp
		 */
		class BASE_IMPEXP CTimeLogger : public mrpt::utils::COutputLogger
		{
		private:
			CTicTac		m_tictac;
			bool		m_enabled;
			std::string m_name;

			//! Data of all the calls:
			struct BASE_IMPEXP TCallData
			{
				TCallData();

				size_t n_calls;
				double min_t,max_t,mean_t,last_t;
				std::stack<double,std::vector<double> >   open_calls;
				bool has_time_units;
			};

			typedef mrpt::utils::ts_hash_map<std::string, TCallData, 1 /* bytes hash */, 10 /* allowed hash collisions */> TDataMap;
			TDataMap  m_data;  //Was: std::map<std::string,TCallData>  m_data;

			void do_enter( const char *func_name );
			double do_leave( const char *func_name );

		public:
			/** Data of each call section: # of calls, minimum, maximum, average and overall execution time (in seconds) \sa getStats */
			struct BASE_IMPEXP TCallStats
			{
				size_t n_calls;
				double min_t,max_t,mean_t,total_t,last_t;
			};

			CTimeLogger(bool enabled=true, const std::string& name=""); //! Default constructor
			virtual ~CTimeLogger(); //!< Destructor

			// We must define these 4 because of the definition of a virtual dtor (compiler will not generate the defaults)
			CTimeLogger(const CTimeLogger&o);
			CTimeLogger &operator =(const CTimeLogger&o);
#if MRPT_HAS_CXX11
			CTimeLogger(CTimeLogger&&o);
			CTimeLogger &operator =(CTimeLogger&&o);
#endif

			std::string getStatsAsText(const size_t column_width=80) const; //!< Dump all stats to a multi-line text string. \sa dumpAllStats, saveToCVSFile
			void getStats(std::map<std::string,TCallStats> &out_stats) const; //!< Returns all the current stats as a map: section_name => stats. \sa getStatsAsText, dumpAllStats, saveToCVSFile
			void dumpAllStats(const size_t column_width=80) const; //!< Dump all stats through the COutputLogger interface. \sa getStatsAsText, saveToCVSFile
			void clear(bool deep_clear=false); //!< Resets all stats. By default (deep_clear=false), all section names are remembered (not freed) so the cost of creating upon the first next call is avoided.
			void enable(bool enabled = true) { m_enabled = enabled; }
			void disable() { m_enabled = false; }
			bool isEnabled() const { return m_enabled;}
			void saveToCSVFile(const std::string &csv_file)  const; 	//!< Dump all stats to a Comma Separated Values (CSV) file. \sa dumpAllStats
			void registerUserMeasure(const char *event_name, const double value);

			void setName(const std::string& name) { m_name =  name; }

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
			/** Return the last execution time of the given "section", or 0 if it hasn't ever been called "enter" with that section name */
			double getLastTime(const std::string &name) const;
		}; // End of class def.


		/** A safe way to call enter() and leave() of a mrpt::utils::CTimeLogger upon construction and destruction of
		 * this auxiliary object, making sure that leave() will be called upon exceptions, etc.
		 * Usage:
		 * \code
		 *    CTimeLogger logger;
		 *    // ...
		 *    { // Start of scope to be monitorized
		 *       CTimeLoggerEntry tle(logger,"operation-name");
		 *
		 *       // do whatever
		 *
		 *    } // End of scope
		 * \endcode
		 * \ingroup mrpt_base_grp
		 */
		struct BASE_IMPEXP CTimeLoggerEntry
		{
			CTimeLoggerEntry(CTimeLogger &logger, const char*section_name );
			~CTimeLoggerEntry();
			CTimeLogger &m_logger;
			const char *m_section_name;
		};


		/** @name Auxiliary stuff for the global profiler used in MRPT_START / MRPT_END macros.
		  @{ */
		extern CTimeLogger BASE_IMPEXP global_profiler;
		void BASE_IMPEXP global_profiler_enter(const char *func_name) MRPT_NO_THROWS;
		void BASE_IMPEXP global_profiler_leave(const char *func_name) MRPT_NO_THROWS;
		/** @} */

	} // End of namespace
} // End of namespace
#endif
