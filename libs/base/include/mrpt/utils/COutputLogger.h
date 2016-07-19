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
#include <mrpt/system/os.h>
#include <mrpt/utils/mrpt_macros.h>

#include <string>
#include <map>
#include <vector>
#include <mrpt/system/datetime.h>
#include <sstream>
#include <iostream>
#include <cstdarg> // for logFmt

namespace mrpt { namespace utils {

/** \brief Enumeration of available verbosity levels */
enum VerbosityLevel { LVL_DEBUG=0, LVL_INFO, LVL_WARN, LVL_ERROR };


/** \brief Versatile class for consistent logging and 
 *        management of output messages
 *
 * COutputLogger is a versatile class for logging messages either to the
 * terminal window or to an external file. Class instances can take messages in
 * std::string using the log class methods 
 *
 * - Logger instance keeps the messages in an internal std::vector so that upon
 *   request it can dump them either to the console or to an external file
 *   altogether.  
 * - The message, when printed in the terminal window, is **colored** according to
 *   the logger's current verbosity/logging level (Logging level with which
 *   the underlying TMsg instance was instantiatedd).  The available verbosity
 *   levels as well as their corresponding colors are listed below:
 *
 *   + LVL_DEBUG => CONCOL_BLUE
 *   + LVL_INFO  => CONCOL_NORMAL 
 *   + LVL_WARN  => CONCOL_GREEN
 *   + LVL_ERROR => CONCOL_RED
 *
 * - Logged messages are displayed in the screen if the current logger level is
 *   higher than m_min_verbosity_level (logger ignores those messages
 *   altogether). This can be used for filtering the output messages according
 *   to their importance (e.g. show only error messages by issuing
 *   setMinLoggingLevel(LVL_ERROR)).
 *   \sa setLoggingLevel, setMinLoggingLevel
 *
 * \note By default every logged message is going to be dumped to the stadard
 * output as well (if VerbisityLevel > m_min_verbosity_level). Unset \b
 * print_message_automatically class variable if that's not the desired
 * behavior
 *
 * \sa TMsg
 * \ingroup mrpt_base_grp
 */
class BASE_IMPEXP COutputLogger {
	public:
		/**
		 * \brief Construct a COutputLogger instance with the given name as the
		 * instance name.
		 *
		 * Call to this constructor can be used instead of first initializing the
		 * object and then explicitly setting the name like in the following case:
		 * \code
		 * COutputLogger a_logger;
		 * a_logger.setName("logger_name");
		 * \endcode
		 */
		COutputLogger(std::string name);
		/** \brief Default class constructor.
		 *
		 * Name of the logger is initialized to *Logger*
		 */
		COutputLogger();
		/**
		 * \brief Default class destructor
		 */
		virtual ~COutputLogger();  //!< virtual dtor (so we can derive classes from this one)

		/** \brief Main method to add the specified message string to the logger.
		 *
		 * Message is printed in the terminal window if
		 * <b>print_message_automatically</b> is set to true. By default it is
		 * saved in the COutputLogger history.
		 *
		 * \sa logCond, logFmt
		 */
		void log(const std::string& msg_str);
		/** \brief Alternative of the previous method, which lets the user specify
		 *         a *temporary logging level*.
		 *
		 * Level will be reverted to the prior logging level
		 * after opration
		 * \sa logCond, logFmt
		 */
		void log(const std::string& msg_str, const VerbosityLevel& level);
		/** \brief Alternative logging method, *handy for usage in const
		 *         functions/methods*
		 *
		 * Used *solely for printing* the given string in the console (but does not
		 * store it internally)
		 * \sa logCond, logFmt
		 */
		void log(const std::string& msg_str, const VerbosityLevel& level) const;
		/** \brief Alternative logging method, handy for usage in const functions/methdos.
		 *
		 * Used solely for printing the given string in the console (but does not
		 * store it internally
		 * \sa logCond, logFmt
		 */
		void log(const std::string& msg_str) const;
		/** \brief Alternative logging method, which mimics the printf behavior.
		 *
		 * Handy for not having to first use mrpt::format to pass a std::string
		 * message to log
		 *
		 * \code
		 * // instead of:
		 * log(mrpt::format("Today is the %d of %s, %d", 15, "July", 2016));
		 *
		 * // one can use:
		 * logFmt("Today is the %d of %s, %d", 15, "July", 2016);
		 * \endcode
		 *
		 * \sa log, logCond, mrpt::utils:CDebugOutputCapable
		 */
		void logFmt(const char* fmt, ...) MRPT_printf_format_check(2,3);
		/** \brief Alternative logging method, which mimics the printf behavior.
		 *
		 * Handy for not having to first use mrpt::format to pass a std::string
		 * message to log. Used *solely for printing* the given string in the console
		 * (but does not store it internally)
		 *
		 * \sa log, logCond, mrpt::utils:CDebugOutputCapable
		 */
		void logFmt(const char* fmt, ...) const MRPT_printf_format_check(2,3);
		/** \brief Log the given message only if the condition is satisfied.
		 *
		 * Level will be reverted to the prior logging level
		 * after opration
		 *
		 * \sa log, logFmt
		 */
		void logCond(const std::string& msg_str, bool cond);
		/** \brief Log the given message only if the condition is satisfied.
		 *
		 * Used *solely for printing* the given string in the console (but does not
		 * store it internally)
		 *
		 * \sa log, logFmt
		 */
		void logCond(const std::string& msg_str, bool cond) const;
		/** \brief Log the given message only if the condition is satisfied.
		 *
		 * \sa log, logFmt
		 */
		void logCond(const std::string& msg_str, const VerbosityLevel& level, bool cond);
		/** \brief Log the given message only if the condition is satisfied.
		 *
		 * Used *solely for printing* the given string in the console (but does not
		 * store it internally)
		 * \sa log, logFmt
		 */
		void logCond(const std::string& msg_str, const VerbosityLevel& level, bool cond) const;
		/** \brief Set the name of the COutputLogger instance 
		 *
		 * \sa getName
		 */
		void setName(const std::string& name);
		/** \brief Return the name of the COutputLogger instance 
		 *
		 * \sa setName
		 */
		std::string getName() const;
		/** \brief Set the logging level that is going to be assigned in each message
		 *         logged from this point on.
		 *
		 * \sa setMinLoggingLevel, getLoggingLevel
		 */
		void setLoggingLevel(const VerbosityLevel& level = LVL_INFO);
		/** \brief Set the *minimum* logging level for which the incoming logs are going to
		 *         be taken into account.
		 *
		 * String messages with specified VerbosityLevel smaller than the min, will
		 * not be outputted to the screen and neither will a record of them be
		 * stored in by the COutputLogger instance
		 *
		 * \sa setLoggingLevel
		 */
		void setMinLoggingLevel(const VerbosityLevel& level = LVL_INFO);
		/** \brief Return the currently used VerbosityLevel 
		 *
		 * \sa setLoggingLevel
		 */
		VerbosityLevel getLoggingLevel() const;
		/** \brief Fill the provided string with the contents of the logger's
		 * history in std::string representation
		 *
		 */
		void getAsString(std::string* fname) const;
		/** \brief Get the history of COutputLogger instance in a string representation.
		 */
		std::string getAsString() const;
		/** \brief Write the contents of the COutputLogger instance to an external file.
		 *
		 * Upon call to this method, COutputLogger dumps the contents of all the
		 * logged commands so far to the specified external file.  By default the
		 * filename is set to ${LOGGERNAME}.log except if the fname parameter is
		 * provided
		 *
		 * \sa dumpToConsole, getAsString
		 */
		void writeToFile(const std::string* fname_in=NULL) const;
		/** \brief Dump the current contents of the COutputLogger instance in the
		 * terminal window.
		 *
			 * \sa writeToFile
		 */
		void dumpToConsole() const;
		/** \brief Return the last Tmsg instance registered in the logger history 
		 *
		 */
		std::string getLastMsg() const;
		/** \brief Fill inputtted string with the contents of the last message in history
		 */
		void getLastMsg(std::string* msg_str) const;
		/** \brief Reset the contents of the logger instance.
		 *
		 * Used *by default* when an instance of the class is initialized
		 */
		void reset();

		/**
		 * \brief Struct responsible of holding information relevant to the message
		 *        (in std::string form) issued by the user.
		 *
		 * Upon TMsg initialization, instance fetches the name of the caller
		 * COutputLogger, as well as the VerbosityLevel and the
		 * mrpt::system::TTimeStamp of the message provided.
		 * The format of the message when this is printed / or written to an
		 * external file complies is given below:
		 *
		 * <center><em> [name | level | timestamp:] body </em></center>
		 */
		struct BASE_IMPEXP TMsg {
			/** \brief Class constructor that passes a message in std::string
			 * form as well as a reference to the COutputLogger that provided the
			 * current message
			 */
			TMsg(const std::string& msg, const COutputLogger& logger);
			/** \brief  Default Destructor */
			~TMsg();

			/** \brief Return a string representation of the underlying message */
			std::string getAsString() const;
			/** \brief Fill the string with the contents of the underlying message in
			 * string representation */
			void getAsString(std::string* contents) const;
			/** \brief Write the message contents to the specified stream
			 *
			 * \sa getAsString
			 */
			void writeToStream(mrpt::utils::CStream& out) const;
			/** \brief Dump the message contents to the standard output
			 *
			 * \sa writeToStream
			 */
			void dumpToConsole() const;
			/** \brief Reset the contents of the TMsg instance */
			void reset();
			/** \brief Return the color corresponding to the given VerbosityLevel
			 *
			 * \sa getLoggingLevelName
			 */
			mrpt::system::TConsoleColor getConsoleColor(VerbosityLevel level) const;
			/** \brief Return the name corresponding to the given VerbosityLevel 
			 *
			 * \sa getConsoleColor
			 */
			std::string getLoggingLevelName(VerbosityLevel level) const; 

			//
			// parameters of the message under construction
			//
			mrpt::system::TTimeStamp timestamp; /**< Timestamp of the message. */
			VerbosityLevel level; /**< Verbosity level of the message. */
			std::string name; /**< Name of the COutputLogger instance that called registered the message. */
			std::string body; /**< Actual content of the message. */

			/** \brief Map from VerbosityLevels to their corresponding mrpt::system::TConsoleColor
			 *
			 * Handy for coloring the input based on the verbosity of the message
			 */
 			std::map<VerbosityLevel, mrpt::system::TConsoleColor> m_levels_to_colors;
			/** \brief Map from VerbosityLevels to their corresponding names
			 *
			 * Handy for printing the current message VerbosityLevel along with the
			 * actual content
			 */
			std::map<VerbosityLevel, std::string> m_levels_to_names;
		};

		/** \brief Set it to false in case you don't want the logged message to be
		 *         dumped to the output automatically. 
		 *
		 * By default it is set to true.
		 */
		bool print_message_automatically;
	private:
		/** Warn (at least for the first usages) that the logged messages are not
		 * stored by the COutputLogger instance since the call to
		 * COutputLogger::log is made inside a const method/function
		 */
		void warnForLogConstMethod() const;

		/** Helper method for generating a std::string instance from printf-like
		 * arguments
		 */
		std::string generateStringFromFormat(const char* fmt, va_list argp) const;

		std::string m_name;
		std::vector<TMsg> m_history;
		VerbosityLevel m_curr_level;
		/** \brief Provided messages with VerbosityLevel smaller than this value shall be ignored */
		VerbosityLevel m_min_verbosity_level;
		
		// how many times have I warned the user about using the const log method
		// if it passes a predefined threshold of times stop polluting the console
		static size_t m_times_for_log_const;
		static size_t m_times_for_log_const_thresh;
};

} }  // END OF NAMESPACES

#endif /* end of include guard: COUTPUTLOGGER_H */
