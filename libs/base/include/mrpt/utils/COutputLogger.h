/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef COUTPUTLOGGER_H
#define COUTPUTLOGGER_H

#include <mrpt/base/link_pragmas.h>
#include <mrpt/utils/mrpt_macros.h>
#include <mrpt/utils/TEnumType.h>
#include <mrpt/system/os.h>  // for console color constants

#include <string>
#include <deque>
#include <mrpt/system/datetime.h>
#include <sstream>
#include <set>

namespace mrpt { namespace utils {
	class CStream; // frwd decl

/** \brief Enumeration of available verbosity levels. \sa COutputLogger */
enum VerbosityLevel {
	LVL_DEBUG=0,
	LVL_INFO,
	LVL_WARN,
	LVL_ERROR,
	// ------------
	NUMBER_OF_VERBOSITY_LEVELS
};

/** Callback types for use with mrpt::utils::COuputLogger */
typedef void (*output_logger_callback_t)(const std::string &msg, const mrpt::utils::VerbosityLevel level, const std::string &loggerName, const mrpt::system::TTimeStamp timestamp, void *userParam);

/** \brief Versatile class for consistent logging and
 *        management of output messages
 *
 * COutputLogger is a versatile class for logging messages either to the
 * terminal window or to an external file. Class instances can take messages in
 * std::string using the logStr class methods. The following macros are also provided
 * for usage within a class that inherits from COutputLogger:
 *
 * \code
 * // Plain strings:
 * MRPT_LOG_DEBUG("This will be shown only if verbosity level is LVL_DEBUG.");
 * MRPT_LOG_ERROR("This message will be always shown.");
 * // printf-like versions:
 * MRPT_LOG_ERROR_FMT("Out of range value: %i.", int_param);
 * // stream-like versions:
 * MRPT_LOG_ERROR_STREAM << "Out of range value: " << int_param;
 * \endcode
 *
 * - Logger instance keeps the messages in an internal container so that upon
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
 *   Default logging level is LVL_INFO.
 *
 * User may receive callbacks whenever a message is displayed to console by using
 * logRegisterCallback(). If for some reason the callbacks are not needed any more,
 * use logDeregisterCallback() to stop receiving calls. This mechanism is useful
 * in case of showing the messages to a GUI, transmiting them to a remote machine, etc.
 *
 * \note By default every logged message is going to be dumped to the standard
 * output as well (if VerbosityLevel > m_min_verbosity_level). Unset \b
 * logging_enable_console_output class variable if that's not the desired
 * behavior
 *
 * \note [New in MRPT 1.5.0]
 * \sa TMsg
 * \ingroup mrpt_base_grp
 */
class BASE_IMPEXP COutputLogger {
	public:
		static mrpt::system::TConsoleColor logging_levels_to_colors[NUMBER_OF_VERBOSITY_LEVELS];  //! Map from VerbosityLevels to their corresponding mrpt::system::TConsoleColor. Handy for coloring the input based on the verbosity of the message
		static std::string logging_levels_to_names[NUMBER_OF_VERBOSITY_LEVELS]; //!< Map from VerbosityLevels to their corresponding names. Handy for printing the current message VerbosityLevel along with the actual content

		/** @name Logging methods
		  * @{ */

		/**
		 * \brief Construct a COutputLogger instance with the given name as the
		 * instance name.
		 *
		 * Call to this constructor can be used instead of first initializing the
		 * object and then explicitly setting the name like in the following case:
		 * \code
		 * COutputLogger a_logger;
		 * a_logger.setLoggerName("logger_name");
		 * \endcode
		 */
		COutputLogger(const std::string &name);
		COutputLogger(); //!< Default class constructor. Name of the logger is initialized to "logStr"
		virtual ~COutputLogger();  //!< virtual dtor (so we can derive classes from this one)

		/** \brief Main method to add the specified message string to the logger.
		 * \sa logCond, logFmt */
		void logStr(const VerbosityLevel level, const std::string& msg_str) const;   // renamed from log() to avoid conflict with math ::log()

		/** \brief Alternative logging method, which mimics the printf behavior.
		 *
		 * Handy for not having to first use mrpt::format to pass a std::string
		 * message to logStr
		 *
		 * \code
		 * // instead of:
		 * logStr(mrpt::format("Today is the %d of %s, %d", 15, "July", 2016));
		 *
		 * // one can use:
		 * logFmt("Today is the %d of %s, %d", 15, "July", 2016);
		 * \endcode
		 *
		 * \sa logStr, logCond
		 */
		void logFmt(const VerbosityLevel level, const char* fmt, ...) const MRPT_printf_format_check(3,4);  // arg 1=this

		/** \brief Log the given message only if the condition is satisfied.
		 *
		 * \sa log, logFmt
		 */
		void logCond(const VerbosityLevel level, bool cond, const std::string& msg_str) const;

		void setLoggerName(const std::string& name);  //!< Set the name of the COutputLogger instance.  \sa getLoggerName
		std::string getLoggerName() const; //!< Return the name of the COutputLogger instance.  \sa setLoggerName

		/** \brief Set the *minimum* logging level for which the incoming logs are going to be taken into account.
		 *
		 * String messages with specified VerbosityLevel smaller than the min, will
		 * not be outputted to the screen and neither will a record of them be
		 * stored in by the COutputLogger instance
		 */
		void setMinLoggingLevel(const VerbosityLevel level);
		/** alias of setMinLoggingLevel() */
		void setVerbosityLevel(const VerbosityLevel level);

		/** \sa setMinLoggingLevel */
		VerbosityLevel getMinLoggingLevel() const { return m_min_verbosity_level; }
		bool isLoggingLevelVisible(VerbosityLevel level) const { return m_min_verbosity_level<=level; }

		void getLogAsString(std::string& log_contents) const; //!< Fill the provided string with the contents of the logger's history in std::string representation
		std::string getLogAsString() const; //!< Get the history of COutputLogger instance in a string representation.

		/** \brief Write the contents of the COutputLogger instance to an external file.
		 *
		 * Upon call to this method, COutputLogger dumps the contents of all the
		 * logged commands so far to the specified external file.  By default the
		 * filename is set to ${LOGGERNAME}.log except if the fname parameter is
		 * provided
		 *
		 * \sa dumpToConsole, getAsString
		 */
		void writeLogToFile(const std::string* fname_in=NULL) const;
		/** \brief Dump the current contents of the COutputLogger instance in the
		 * terminal window.
		 *
		 * \sa writeToFile
		 */
		void dumpLogToConsole() const;
		std::string getLoggerLastMsg() const;  //!< Return the last Tmsg instance registered in the logger history
		void getLoggerLastMsg(std::string& msg_str) const; //!< Fill inputtted string with the contents of the last message in history
		void loggerReset(); //!< Reset the contents of the logger instance. Called upon construction.

		bool logging_enable_console_output; //!< [Default=true] Set it to false in case you don't want the logged messages to be dumped to the output automatically.
		bool logging_enable_keep_record;    //!< [Default=false] Enables storing all messages into an internal list. \sa writeLogToFile, getLogAsString

		void logRegisterCallback(output_logger_callback_t  userFunc, void *userParam = NULL);
		void logDeregisterCallback(output_logger_callback_t  userFunc, void *userParam = NULL);
		/** @} */

		struct BASE_IMPEXP TCallbackEntry
		{
			output_logger_callback_t  func;
			void *userParam;

			bool operator <(const mrpt::utils::COutputLogger::TCallbackEntry &e2) const {
				return func<e2.func;
			}
			bool operator == (const mrpt::utils::COutputLogger::TCallbackEntry &c2) const {
				return func == c2.func && userParam == c2.userParam;
			}

		};

	protected:
		/** \brief Provided messages with VerbosityLevel smaller than this value shall be ignored */
		VerbosityLevel m_min_verbosity_level;
	private:
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
			TMsg(const mrpt::utils::VerbosityLevel level, const std::string& msg, const COutputLogger& logger);
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

			// parameters of the message under construction
			mrpt::system::TTimeStamp timestamp; /**< Timestamp of the message. */
			VerbosityLevel level; /**< Verbosity level of the message. */
			std::string name; /**< Name of the COutputLogger instance that called registered the message. */
			std::string body; /**< Actual content of the message. */
		};

		std::string generateStringFromFormat(const char* fmt, va_list argp) const; //!< Helper method for generating a std::string instance from printf-like arguments

		std::string m_logger_name;
		mutable std::deque<TMsg> m_history;   // deque is better than vector to avoid memory reallocs

		std::set<TCallbackEntry> m_listCallbacks;
};

	/** For use in MRPT_LOG_DEBUG_STREAM, etc. */
	struct BASE_IMPEXP COutputLoggerStreamWrapper
	{
		COutputLoggerStreamWrapper(VerbosityLevel level, const COutputLogger &logger) : m_level(level),m_logger(logger) {}
		~COutputLoggerStreamWrapper() { if (m_logger.isLoggingLevelVisible(m_level)) m_logger.logStr(m_level, m_str.str()); }

		template <typename T>
		std::stringstream & operator << (const T &val) {
			m_str << val;
			return m_str;
		}
		// Overload for std::stringstream objects
		std::stringstream & operator << (const std::stringstream &val) {
			m_str << val.str();
			return m_str;
		}

	private:
		std::stringstream m_str;
		VerbosityLevel m_level;
		const COutputLogger &m_logger;
	};

#define MRPT_LOG_DEBUG(_STRING) this->logStr(::mrpt::utils::LVL_DEBUG, _STRING)
#define MRPT_LOG_INFO(_STRING) this->logStr(::mrpt::utils::LVL_INFO, _STRING)
#define MRPT_LOG_WARN(_STRING) this->logStr(::mrpt::utils::LVL_WARN, _STRING)
#define MRPT_LOG_ERROR(_STRING) this->logStr(::mrpt::utils::LVL_ERROR, _STRING)

#define MRPT_LOG_DEBUG_FMT(_FMT_STRING,...) this->logFmt(::mrpt::utils::LVL_DEBUG, _FMT_STRING, __VA_ARGS__)
#define MRPT_LOG_INFO_FMT(_FMT_STRING,...) this->logFmt(::mrpt::utils::LVL_INFO, _FMT_STRING, __VA_ARGS__)
#define MRPT_LOG_WARN_FMT(_FMT_STRING,...) this->logFmt(::mrpt::utils::LVL_WARN, _FMT_STRING, __VA_ARGS__)
#define MRPT_LOG_ERROR_FMT(_FMT_STRING,...) this->logFmt(::mrpt::utils::LVL_ERROR, _FMT_STRING, __VA_ARGS__)

/** Usage: `MRPT_LOG_DEBUG_STREAM << "Var=" << value;` */
#define MRPT_LOG_DEBUG_STREAM ::mrpt::utils::COutputLoggerStreamWrapper(::mrpt::utils::LVL_DEBUG,*this)
#define MRPT_LOG_INFO_STREAM ::mrpt::utils::COutputLoggerStreamWrapper(::mrpt::utils::LVL_INFO,*this)
#define MRPT_LOG_WARN_STREAM ::mrpt::utils::COutputLoggerStreamWrapper(::mrpt::utils::LVL_WARN,*this)
#define MRPT_LOG_ERROR_STREAM ::mrpt::utils::COutputLoggerStreamWrapper(::mrpt::utils::LVL_ERROR,*this)

}
	// Specializations MUST occur at the same namespace:
	namespace utils
	{
		template <>
		struct TEnumTypeFiller<mrpt::utils::VerbosityLevel>
		{
			typedef mrpt::utils::VerbosityLevel enum_t;
			static void fill(bimap<enum_t,std::string>  &m_map)
			{
				using namespace mrpt::utils;
				m_map.insert(LVL_DEBUG,          "DEBUG");
				m_map.insert(LVL_DEBUG,          "LVL_DEBUG");
				m_map.insert(LVL_INFO ,          "INFO");
				m_map.insert(LVL_INFO ,          "LVL_INFO");
				m_map.insert(LVL_WARN ,          "WARN");
				m_map.insert(LVL_WARN ,          "LVL_WARN");
				m_map.insert(LVL_ERROR,          "ERROR");
				m_map.insert(LVL_ERROR,          "LVL_ERROR");
			}
		};
	} // End of namespace
}
#endif /* end of include guard: COUTPUTLOGGER_H */
