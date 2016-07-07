/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef COUTPUTLOGGER_H
#define COUTPUTLOGGER_H

#include <mrpt/utils/CStream.h>
#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/utils/CStdOutStream.h>
#include <mrpt/system/os.h>
#include <mrpt/system/threads.h>

#include <string>
#include <map>
#include <vector>
#include <mrpt/system/datetime.h>
#include <sstream>
#include <iostream>

namespace mrpt { namespace utils {

// enumeration of available verbosity levels
enum VerbosityLevel { LVL_DEBUG=0, LVL_INFO, LVL_WARN, LVL_ERROR };


class COutputLogger_t {
  public:
    COutputLogger_t(std::string name);
    COutputLogger_t();
    ~COutputLogger_t();

		/*
		 * Main method to add a specific method to the logger
		 * It is printed in the terminal window if dump_to_terminal is set
		 * to true. By default it is saved in the COutputLogger_t history.
		 */
    void log(const std::string& msg_str);
    /** 
     * Alternative of the previous method, which lets the user specify a temporary
     * logging level. Level will be reverted to the prior logging level
     * after opration
     */
    void log(const std::string& msg_str, const VerbosityLevel& level);
    /**
     * log the given message only if the condition is satisfied
     */
    void logCond(const std::string& msg_str, bool cond);
    /**
     * log the given message only if the condition is satisfied
     */
    void logCond(const std::string& msg_str, const VerbosityLevel& level, bool cond);

		void setName(const std::string& name);
		std::string getName() const;
		/*
		 * Set the logging level that is going to be assigned in each message
		 * logged from this point on.
		 */
    void setLoggingLevel(const VerbosityLevel& level = LVL_INFO);
    /** Set the minimum logging level for which the incoming logs are going to
     * be taken into account
     */
    void setMinLoggingLevel(const VerbosityLevel& level = LVL_INFO);

    VerbosityLevel getCurrentLoggingLevel() const;
    /**
     * Write the contents of the COutputLogger_t instance to an external file.
     * By default the filename is set to LOGGERNAME.log except if the fname
     * parameter is provided
     */
    void writeToFile(const std::string* fname_in=NULL) const;
    /**
     * Dump the current contents of the COutputLogger_t instance in the terminal
     * window
     */
    void dumpToConsole() const;
		/**
		 * Return the last Tmsg instance in the logger history
		 */
		std::string getLastMsg() const;
		/**
		 * Fill inputtted string with the contents of the last message in history
		 */
		void getLastMsg(std::string* msg_str) const;

    /**
     * Reset the state of the logger.
     */
    void reset();

    struct TMsg {
    	TMsg(const std::string& msg, const COutputLogger_t& logger);
    	~TMsg();

			std::string getAsString() const;
			void getAsString(std::string* contents) const;
			void writeToStream(mrpt::utils::CStream& out) const;
			void dumpToConsole() const;
			// reset the contents of the TMsg instance
			void reset();
			/**
		 	 * Return the color corresponding to the given VerbosityLevel
		 	 */
			mrpt::system::TConsoleColor getConsoleColor(VerbosityLevel level) const;
			/**
		 	 * Return the name corresponding to the given VerbosityLevel
		 	 */
			std::string getLoggingLevelName(VerbosityLevel level) const; 

    	//
    	// parameters of the message under construction
    	// Message format
    	// [name | level | timestamp:] body
    	//
			mrpt::system::TTimeStamp timestamp;
			VerbosityLevel level;
			std::string name;
			std::string body;
			mrpt::system::TConsoleColor color;
			std::string level_name;

 			std::map<VerbosityLevel, mrpt::system::TConsoleColor> m_levels_to_colors;
			std::map<VerbosityLevel, std::string> m_levels_to_names;
   };

  private:
		std::string m_name;
		std::vector<TMsg> m_history;
		VerbosityLevel m_curr_level;
		// logs with VerbosityLevel smaller than this value shall be ignored
		VerbosityLevel m_min_verbosity_level;

		mrpt::system::TTimeStamp m_curr_timestamp;

		bool m_print_message_automatically;
};

} }  // END OF NAMESPACES

#endif /* end of include guard: COUTPUTLOGGER_H */
