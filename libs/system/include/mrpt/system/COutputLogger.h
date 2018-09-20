/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/typemeta/TEnumType.h>
#include <mrpt/system/os.h>  // for console color constants
#include <mrpt/system/CTicTac.h>
#include <mrpt/core/Clock.h>

#include <string>
#include <deque>
#include <array>
#include <sstream>
#include <iosfwd>
#include <functional>

namespace mrpt::system
{
/** \brief Enumeration of available verbosity levels. \sa COutputLogger */
enum VerbosityLevel
{
	LVL_DEBUG = 0,
	LVL_INFO,
	LVL_WARN,
	LVL_ERROR,
	// ------------
	NUMBER_OF_VERBOSITY_LEVELS
};

/** Callback types for use with mrpt::system::COuputLogger */
using output_logger_callback_t = std::function<void(
	const std::string& msg, const mrpt::system::VerbosityLevel level,
	const std::string& loggerName, const mrpt::Clock::time_point& timestamp)>;

/** \brief Versatile class for consistent logging and
 *        management of output messages
 *
 * COutputLogger is a versatile class for logging messages either to the
 * terminal window or to an external file. Class instances can take messages in
 * std::string using the logStr class methods. The following macros are also
 * provided
 * for usage within a class that inherits from COutputLogger:
 *
 * \code
 * // Plain strings:
 * MRPT_LOG_DEBUG("This will be shown only if verbosity level is LVL_DEBUG.");
 * MRPT_LOG_ERROR("This message will be always shown.");
 * // printf-like versions:
 * MRPT_LOG_ERROR_FMT("Out of range value: %i.", int_param);
 * // stream-like versions:
 * MRPT_LOG_ERROR_STREAM("Out of range value: " << int_param << " more vars: "
 * << other_var);
 * // Minimum period (in seconds) between messages:
 * MRPT_LOG_THROTTLE_DEBUG_STREAM(5.0, "Var=" << value << " foo=" << foo_var);
 * // Only once:
 * MRPT_LOG_ONCE_WARN("Notice: blah blah");
 * \endcode
 *
 * From outside of a class inheriting from COutputLogger the following
 * `MRPT_UNSCOPED_{START|END}` macros are provided:
 *
 *  \code
 *  MRPT_UNSCOPED_LOGGER_START;
 *  MRPT_LOG_WARN("blah");
 *  MRPT_LOG_ERROR_STREAM("error: " << strval);
 *  MRPT_UNSCOPED_LOGGER_END;
 *  \endcode
 *
 * - Logger instance keeps the messages in an internal container so that upon
 *   request it can dump them either to the console or to an external file
 *   altogether.
 * - The message, when printed in the terminal window, is **colored** according
 * to
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
 * User may receive callbacks whenever a message is displayed to console by
 * using
 * logRegisterCallback(). If for some reason the callbacks are not needed any
 * more,
 * use logDeregisterCallback() to stop receiving calls. This mechanism is useful
 * in case of showing the messages to a GUI, transmiting them to a remote
 * machine, etc.
 *
 * \note By default every logged message is going to be dumped to the standard
 * output as well (if VerbosityLevel > m_min_verbosity_level). Unset \b
 * logging_enable_console_output class variable if that's not the desired
 * behavior
 *
 * \note [New in MRPT 1.5.0]
 * \sa TMsg
 * \ingroup mrpt_system_grp
 */
class COutputLogger
{
   public:
	/** Map from VerbosityLevels to their corresponding
	 * mrpt::system::TConsoleColor. Handy for coloring the input based on the
	 * verbosity of the message */
	static std::array<mrpt::system::TConsoleColor, NUMBER_OF_VERBOSITY_LEVELS>&
		logging_levels_to_colors();

	/** Map from VerbosityLevels to their corresponding names. Handy for
	 * printing the current message VerbosityLevel along with the actual content
	 */
	static std::array<std::string, NUMBER_OF_VERBOSITY_LEVELS>&
		logging_levels_to_names();

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
	COutputLogger(const std::string& name);
	/** Default class constructor. Name of the logger is initialized to "logStr"
	 */
	COutputLogger();
	/** virtual dtor (so we can derive classes from this one) */
	virtual ~COutputLogger();

	/** \brief Main method to add the specified message string to the logger.
	 * \sa logCond, logFmt */
	void logStr(const VerbosityLevel level, const std::string& msg_str)
		const;  // renamed from log() to avoid conflict with math ::log()

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
	void logFmt(const VerbosityLevel level, const char* fmt, ...) const
		MRPT_printf_format_check(3, 4);  // arg 1=this

	/** \brief Log the given message only if the condition is satisfied.
	 *
	 * \sa log, logFmt
	 */
	void logCond(
		const VerbosityLevel level, bool cond,
		const std::string& msg_str) const;

	/** Set the name of the COutputLogger instance.  \sa getLoggerName */
	void setLoggerName(const std::string& name);
	/** Return the name of the COutputLogger instance.  \sa setLoggerName */
	std::string getLoggerName() const;

	/** \brief Set the *minimum* logging level for which the incoming logs are
	 * going to be taken into account.
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
	bool isLoggingLevelVisible(VerbosityLevel level) const
	{
		return m_min_verbosity_level <= level;
	}

	/** Fill the provided string with the contents of the logger's history in
	 * std::string representation */
	void getLogAsString(std::string& log_contents) const;
	/** Get the history of COutputLogger instance in a string representation. */
	std::string getLogAsString() const;

	/** \brief Write the contents of the COutputLogger instance to an external
	 * file.
	 *
	 * Upon call to this method, COutputLogger dumps the contents of all the
	 * logged commands so far to the specified external file.  By default the
	 * filename is set to ${LOGGERNAME}.log except if the fname parameter is
	 * provided
	 *
	 * \sa dumpToConsole, getAsString
	 */
	void writeLogToFile(const std::string* fname_in = nullptr) const;
	/** \brief Dump the current contents of the COutputLogger instance in the
	 * terminal window.
	 *
	 * \sa writeToFile
	 */
	void dumpLogToConsole() const;
	/** Return the last Tmsg instance registered in the logger history */
	std::string getLoggerLastMsg() const;
	/** Fill inputtted string with the contents of the last message in history
	 */
	void getLoggerLastMsg(std::string& msg_str) const;
	/** Reset the contents of the logger instance. Called upon construction. */
	void loggerReset();

	/** [Default=true] Set it to false in case you don't want the logged
	 * messages to be dumped to the output automatically. */
	bool logging_enable_console_output{true};
	/** [Default=false] Enables storing all messages into an internal list. \sa
	 * writeLogToFile, getLogAsString */
	bool logging_enable_keep_record{false};

	void logRegisterCallback(output_logger_callback_t userFunc);
	/** \return true if an entry was found and deleted. */
	bool logDeregisterCallback(output_logger_callback_t userFunc);
	/** @} */

   protected:
	/** \brief Provided messages with VerbosityLevel smaller than this value
	 * shall be ignored */
	VerbosityLevel m_min_verbosity_level{LVL_INFO};

   private:
	/**
	 * \brief Struct responsible of holding information relevant to the message
	 *        (in std::string form) issued by the user.
	 *
	 * Upon TMsg initialization, instance fetches the name of the caller
	 * COutputLogger, as well as the VerbosityLevel and the
	 * mrpt::Clock::time_point of the message provided.
	 * The format of the message when this is printed / or written to an
	 * external file complies is given below:
	 *
	 * <center><em> [name | level | timestamp:] body </em></center>
	 */
	struct TMsg
	{
		/** \brief Class constructor that passes a message in std::string
		 * form as well as a reference to the COutputLogger that provided the
		 * current message
		 */
		TMsg(
			const mrpt::system::VerbosityLevel level, const std::string& msg,
			const COutputLogger& logger);
		/** \brief  Default Destructor */
		~TMsg();

		/** \brief Return a string representation of the underlying message */
		std::string getAsString() const;
		/** \brief Fill the string with the contents of the underlying message
		 * in
		 * string representation */
		void getAsString(std::string* contents) const;
		/** \brief Write the message contents to the specified stream
		 *
		 * \sa getAsString
		 */
		void writeToStream(std::ostream& out) const;
		/** \brief Dump the message contents to the standard output
		 *
		 * \sa writeToStream
		 */
		void dumpToConsole() const;

		// parameters of the message under construction
		mrpt::Clock::time_point timestamp; /**< Timestamp of the message. */
		VerbosityLevel level; /**< Verbosity level of the message. */
		std::string name; /**< Name of the COutputLogger instance that called
							 registered the message. */
		std::string body; /**< Actual content of the message. */
	};

	/** Helper method for generating a std::string instance from printf-like
	 * arguments */
	std::string generateStringFromFormat(const char* fmt, va_list argp) const;

	std::string m_logger_name;
	mutable std::deque<TMsg>
		m_history;  // deque is better than vector to avoid memory reallocs

	std::deque<output_logger_callback_t> m_listCallbacks;
};

/** For use in MRPT_LOG_DEBUG_STREAM(), etc. */
struct COutputLoggerStreamWrapper
{
	COutputLoggerStreamWrapper(
		VerbosityLevel level, const COutputLogger& logger)
		: m_level(level), m_logger(logger)
	{
	}
	~COutputLoggerStreamWrapper()
	{
		if (m_logger.isLoggingLevelVisible(m_level))
			m_logger.logStr(m_level, m_str.str());
	}

	template <typename T>
	std::stringstream& operator<<(const T& val)
	{
		m_str << val;
		return m_str;
	}
	// Overload for std::stringstream objects
	std::stringstream& operator<<(const std::stringstream& val)
	{
		m_str << val.str();
		return m_str;
	}

   private:
	std::stringstream m_str;
	VerbosityLevel m_level;
	const COutputLogger& m_logger;
};

#define INTERNAL_MRPT_LOG(_LVL, _STRING) this->logStr(_LVL, _STRING)

#define INTERNAL_MRPT_LOG_ONCE(_LVL, _STRING) \
	do                                        \
	{                                         \
		static once_flag = false;             \
		if (!once_flag)                       \
		{                                     \
			once_flag = true;                 \
			this->logStr(_LVL, _STRING);      \
		}                                     \
	} while (0)

#define INTERNAL_MRPT_LOG_FMT(_LVL, _FMT_STRING, ...)     \
	do                                                    \
	{                                                     \
		if (this->isLoggingLevelVisible(_LVL))            \
		{                                                 \
			this->logFmt(_LVL, _FMT_STRING, __VA_ARGS__); \
		}                                                 \
	} while (0)

#define INTERNAL_MRPT_LOG_STREAM(_LVL, __CONTENTS)                  \
	do                                                              \
	{                                                               \
		if (this->isLoggingLevelVisible(_LVL))                      \
		{                                                           \
			::mrpt::system::COutputLoggerStreamWrapper(_LVL, *this) \
				<< __CONTENTS;                                      \
		}                                                           \
	} while (0)

#define INTERNAL_MRPT_LOG_THROTTLE(_LVL, _PERIOD_SECONDS, _STRING) \
	do                                                             \
	{                                                              \
		if (this->isLoggingLevelVisible(_LVL))                     \
		{                                                          \
			static mrpt::system::CTicTac tim;                      \
			if (tim.Tac() > _PERIOD_SECONDS)                       \
			{                                                      \
				tim.Tic();                                         \
				this->logStr(_LVL, _STRING);                       \
			}                                                      \
		}                                                          \
	} while (0)

#define INTERNAL_MRPT_LOG_THROTTLE_STREAM(_LVL, _PERIOD_SECONDS, __CONTENTS) \
	do                                                                       \
	{                                                                        \
		if (this->isLoggingLevelVisible(_LVL))                               \
		{                                                                    \
			static mrpt::system::CTicTac tim;                                \
			if (tim.Tac() > _PERIOD_SECONDS)                                 \
			{                                                                \
				tim.Tic();                                                   \
				::mrpt::system::COutputLoggerStreamWrapper(_LVL, *this)      \
					<< __CONTENTS;                                           \
			}                                                                \
		}                                                                    \
	} while (0)

#define INTERNAL_MRPT_LOG_THROTTLE_FMT(                       \
	_LVL, _PERIOD_SECONDS, _FMT_STRING, ...)                  \
	do                                                        \
	{                                                         \
		if (this->isLoggingLevelVisible(_LVL))                \
		{                                                     \
			static mrpt::system::CTicTac tim;                 \
			if (tim.Tac() > _PERIOD_SECONDS)                  \
			{                                                 \
				tim.Tic();                                    \
				this->logFmt(_LVL, _FMT_STRING, __VA_ARGS__); \
			}                                                 \
		}                                                     \
	} while (0)

/** Use: `MRPT_LOG_DEBUG("message");`  */
#define MRPT_LOG_DEBUG(_STRING) \
	INTERNAL_MRPT_LOG(::mrpt::system::LVL_DEBUG, _STRING)
#define MRPT_LOG_INFO(_STRING) \
	INTERNAL_MRPT_LOG(::mrpt::system::LVL_INFO, _STRING)
#define MRPT_LOG_WARN(_STRING) \
	INTERNAL_MRPT_LOG(::mrpt::system::LVL_WARN, _STRING)
#define MRPT_LOG_ERROR(_STRING) \
	INTERNAL_MRPT_LOG(::mrpt::system::LVL_ERROR, _STRING)

/** Use: `MRPT_LOG_ONCE_DEBUG("once-only message");`  */
#define MRPT_LOG_ONCE_DEBUG(_STRING) \
	INTERNAL_MRPT_LOG_ONCE(::mrpt::system::LVL_DEBUG, _STRING)
#define MRPT_LOG_ONCE_INFO(_STRING) \
	INTERNAL_MRPT_LOG_ONCE(::mrpt::system::LVL_INFO, _STRING)
#define MRPT_LOG_ONCE_WARN(_STRING) \
	INTERNAL_MRPT_LOG_ONCE(::mrpt::system::LVL_WARN, _STRING)
#define MRPT_LOG_ONCE_ERROR(_STRING) \
	INTERNAL_MRPT_LOG_ONCE(::mrpt::system::LVL_ERROR, _STRING)

/** Use: `MRPT_LOG_THROTTLE_DEBUG(5.0, "message");`  */
#define MRPT_LOG_THROTTLE_DEBUG(_PERIOD_SECONDS, _STRING) \
	INTERNAL_MRPT_LOG_THROTTLE(                           \
		::mrpt::system::LVL_DEBUG, _PERIOD_SECONDS, _STRING)
#define MRPT_LOG_THROTTLE_INFO(_PERIOD_SECONDS, _STRING) \
	INTERNAL_MRPT_LOG_THROTTLE(                          \
		::mrpt::system::LVL_INFO, _PERIOD_SECONDS, _STRING)
#define MRPT_LOG_THROTTLE_WARN(_PERIOD_SECONDS, _STRING) \
	INTERNAL_MRPT_LOG_THROTTLE(                          \
		::mrpt::system::LVL_WARN, _PERIOD_SECONDS, _STRING)
#define MRPT_LOG_THROTTLE_ERROR(_PERIOD_SECONDS, _STRING) \
	INTERNAL_MRPT_LOG_THROTTLE(                           \
		::mrpt::system::LVL_ERROR, _PERIOD_SECONDS, _STRING)

/** Use: `MRPT_LOG_DEBUG_FMT("i=%u", i);`  */
#define MRPT_LOG_DEBUG_FMT(_FMT_STRING, ...) \
	INTERNAL_MRPT_LOG_FMT(::mrpt::system::LVL_DEBUG, _FMT_STRING, __VA_ARGS__)
#define MRPT_LOG_INFO_FMT(_FMT_STRING, ...) \
	INTERNAL_MRPT_LOG_FMT(::mrpt::system::LVL_INFO, _FMT_STRING, __VA_ARGS__)
#define MRPT_LOG_WARN_FMT(_FMT_STRING, ...) \
	INTERNAL_MRPT_LOG_FMT(::mrpt::system::LVL_WARN, _FMT_STRING, __VA_ARGS__)
#define MRPT_LOG_ERROR_FMT(_FMT_STRING, ...) \
	INTERNAL_MRPT_LOG_FMT(::mrpt::system::LVL_ERROR, _FMT_STRING, __VA_ARGS__)

/** Use: `MRPT_LOG_DEBUG_STREAM("Var=" << value << " foo=" << foo_var);` */
#define MRPT_LOG_DEBUG_STREAM(__CONTENTS) \
	INTERNAL_MRPT_LOG_STREAM(::mrpt::system::LVL_DEBUG, __CONTENTS)
#define MRPT_LOG_INFO_STREAM(__CONTENTS) \
	INTERNAL_MRPT_LOG_STREAM(::mrpt::system::LVL_INFO, __CONTENTS)
#define MRPT_LOG_WARN_STREAM(__CONTENTS) \
	INTERNAL_MRPT_LOG_STREAM(::mrpt::system::LVL_WARN, __CONTENTS)
#define MRPT_LOG_ERROR_STREAM(__CONTENTS) \
	INTERNAL_MRPT_LOG_STREAM(::mrpt::system::LVL_ERROR, __CONTENTS)

/** Usage: `MRPT_LOG_THROTTLE_DEBUG_STREAM(5.0, "Var=" << value << " foo=" <<
 * foo_var);` */
#define MRPT_LOG_THROTTLE_DEBUG_STREAM(_PERIOD_SECONDS, __CONTENTS) \
	INTERNAL_MRPT_LOG_THROTTLE_STREAM(                              \
		::mrpt::system::LVL_DEBUG, _PERIOD_SECONDS, __CONTENTS)
#define MRPT_LOG_THROTTLE_INFO_STREAM(_PERIOD_SECONDS, __CONTENTS) \
	INTERNAL_MRPT_LOG_THROTTLE_STREAM(                             \
		::mrpt::system::LVL_INFO, _PERIOD_SECONDS, __CONTENTS)
#define MRPT_LOG_THROTTLE_WARN_STREAM(_PERIOD_SECONDS, __CONTENTS) \
	INTERNAL_MRPT_LOG_THROTTLE_STREAM(                             \
		::mrpt::system::LVL_WARN, _PERIOD_SECONDS, __CONTENTS)
#define MRPT_LOG_THROTTLE_ERROR_STREAM(_PERIOD_SECONDS, __CONTENTS) \
	INTERNAL_MRPT_LOG_THROTTLE_STREAM(                              \
		::mrpt::system::LVL_ERROR, _PERIOD_SECONDS, __CONTENTS)

/** Usage: `MRPT_LOG_THROTTLE_DEBUG_FMT(5.0, "i=%u", i);` */
#define MRPT_LOG_THROTTLE_DEBUG_FMT(_PERIOD_SECONDS, _FMT_STRING, ...) \
	INTERNAL_MRPT_LOG_THROTTLE_FMT(                                    \
		::mrpt::system::LVL_DEBUG, _PERIOD_SECONDS, _FMT_STRING, __VA_ARGS__)
#define MRPT_LOG_THROTTLE_INFO_FMT(_PERIOD_SECONDS, _FMT_STRING, ...) \
	INTERNAL_MRPT_LOG_THROTTLE_FMT(                                   \
		::mrpt::system::LVL_INFO, _PERIOD_SECONDS, _FMT_STRING, __VA_ARGS__)
#define MRPT_LOG_THROTTLE_WARN_FMT(_PERIOD_SECONDS, _FMT_STRING, ...) \
	INTERNAL_MRPT_LOG_THROTTLE_FMT(                                   \
		::mrpt::system::LVL_WARN, _PERIOD_SECONDS, _FMT_STRING, __VA_ARGS__)
#define MRPT_LOG_THROTTLE_ERROR_FMT(_PERIOD_SECONDS, _FMT_STRING, ...) \
	INTERNAL_MRPT_LOG_THROTTLE_FMT(                                    \
		::mrpt::system::LVL_ERROR, _PERIOD_SECONDS, _FMT_STRING, __VA_ARGS__)

#ifdef _DEBUG
#define DEFAULT_LOGLVL_MRPT_UNSCOPED ::mrpt::system::LVL_DEBUG
#else
#define DEFAULT_LOGLVL_MRPT_UNSCOPED ::mrpt::system::LVL_DEBUG
#endif

/** For calling any `MRPT_LOG_*()` macro from outside of an object inherited
 * from COutputLogger.
 * Debug level is `DEBUG` if build with `_DEBUG` preprocessor flag, `INFO`
 * otherwise.
 * Use:
 * \code
 *  MRPT_UNSCOPED_LOGGER_START;
 *  MRPT_LOG_WARN("blah");
 *  MRPT_LOG_ERROR_STREAM("error: " << strval);
 *  MRPT_UNSCOPED_LOGGER_END;
 * \endcode
 */
#define MRPT_UNSCOPED_LOGGER_START                                      \
	do                                                                  \
	{                                                                   \
		struct dummy_logger_ : public mrpt::system::COutputLogger       \
		{                                                               \
			dummy_logger_() : mrpt::system::COutputLogger("MRPT_log")   \
			{                                                           \
				this->setMinLoggingLevel(DEFAULT_LOGLVL_MRPT_UNSCOPED); \
			}                                                           \
			void usercode()                                             \
			{                                                           \
				do                                                      \
				{                                                       \
				} while (0)
// Here comes the user code, which is run in the ctor, and will call the object
// log methods.

#define MRPT_UNSCOPED_LOGGER_END  \
	}                             \
	}                             \
	;                             \
	static dummy_logger_ tmp_obj; \
	tmp_obj.usercode();           \
	}                             \
	while (0)
}  // namespace mrpt::system
// TTypeEnum for verbosity levels:
MRPT_ENUM_TYPE_BEGIN_NAMESPACE(mrpt::system, mrpt::system::VerbosityLevel)
MRPT_FILL_ENUM(LVL_DEBUG);
MRPT_FILL_ENUM(LVL_INFO);
MRPT_FILL_ENUM(LVL_WARN);
MRPT_FILL_ENUM(LVL_ERROR);
MRPT_FILL_ENUM_CUSTOM_NAME(LVL_DEBUG, "DEBUG");
MRPT_FILL_ENUM_CUSTOM_NAME(LVL_INFO, "INFO");
MRPT_FILL_ENUM_CUSTOM_NAME(LVL_WARN, "WARN");
MRPT_FILL_ENUM_CUSTOM_NAME(LVL_ERROR, "ERROR");
MRPT_ENUM_TYPE_END()
