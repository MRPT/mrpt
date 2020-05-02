
//  Copyright (c) 2003-2019 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#include "journalstackwalker.h"
#include "journaller.h"
#include "journalthreader.h"
#include "journalfile.h"

#include <map>
#include <fstream>
#include <xstypes/xstime.h>
#include <xstypes/pstdint.h>	// for PRINTF_INT64_MODIFIER
#include <xstypes/xsthread.h>	// for xsGetCurrentThreadId
#include <xstypes/xstimestamp.h>
#include <xstypes/xstimeinfo.h>
#include <memory>

#ifndef XSENS_NO_AUTOLIB
#pragma comment(lib, "psapi.lib")
#endif

//! \brief The optional additional logger
AbstractAdditionalLogger* Journaller::m_additionalLogger = nullptr;

inline int threadId() {
#if JOURNALLER_WITH_THREAD_SUPPORT && !defined(__APPLE__)
	return (int) xsGetCurrentThreadId();
#else
	return 0;
#endif
}

/*! \class Journaller
	\brief A journalling class for debugging applications

	\details Create a Journaller object in your application or dll main and share it among the source files
	of your application (ie by adding "extern Journaller* journal;" in the xscommon_config.h)
	Use the supplied logging macros if you want to be able to remove the logging lines at compile-time.

	Multiple Journaller objects can use the same file
*/

#if __GNUC__ >= 6
// Ignore warning: nonnull argument ‘this’ compared to NULL [-Wnonnull-compare]
// This happens for the JL macro's which here have a this argument, which is always nonnull
// In the macros we compare this to NULL, which is now a warning in GCC6
// On other locations, we do not use this for the macro, hence the warning does not show there
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnonnull-compare"
#endif

/*! \brief Constructor
	\details The constructor configures the object to use the specified file. If it is already open by
	another Journaller object, the existing file is reused and it will not be purged.
	\param pathfile The (path+) filename of the logfile to use
	\param purge Whether to clear the logfile or append when opening the file (default true)
	\param initialLogLevel The initial log level to use (default JLL_Alert)
*/
Journaller::Journaller(const XsString& pathfile, bool purge, JournalLogLevel initialLogLevel)
	: m_file(nullptr)
	, m_level(initialLogLevel)
	, m_debugLevel(initialLogLevel)
	, m_flushLevel(JLL_Alert)
	, m_threader(new JournalThreader)
	, m_useDateTime(false)
{
	init(pathfile, purge);
}

/*! \brief Destructor, detaches from the logfile and closes it if this was the last reference */
Journaller::~Journaller()
{
}

/*! \brief Initialize the Journaller by recreating the internal journal file */
void Journaller::init(XsString const& pathfile, bool purge)
{
#ifdef ANDROID
	setTag(tagFromFilename(pathfile.toStdString()));
#endif
	m_file.reset(new JournalFile(pathfile, purge));
}

/*! \returns The tag string made from file name
	\details This function is primarily useful on Android, where we always
	need a tag set.
	\param fn The file name string
*/
std::string Journaller::tagFromFilename(const std::string &fn)
{
	const size_t b = fn.find_last_of("/") + 1;
	const size_t l = fn.find_first_of(".");
	return fn.substr(b, l - b);
}

/*! \brief When setting the Date Time to \a yes, the timestamps are translated from unix timestamp into a redable date/time
	\param yes The boolean value to set
	\note the translation is not free, the recommended behaviour is to stick with the default unix timestamps
*/
void Journaller::setUseDateTime(bool yes)
{
	m_useDateTime = yes;
}

/*! \brief Sets the additional logger
	\param[in] additionallogger A pointer (may be null) to the additional logger.
	If additionalLogger is a null pointer, the current additionalLogger is removed.
	\note Journaller does NOT take ownership of the logger.
*/
void Journaller::setAdditionalLogger(AbstractAdditionalLogger* additionallogger)
{
	m_additionalLogger = additionallogger;
}

/*! \brief A list of strings representing the different log levels, use JournalLogLevel to index */
static const char* gLogLevelString[] = {
		"[TRACE] ",
		"[DEBUG] ",
		"[ALERT] ",
		"[ERROR] ",
		"[FATAL] ",
		"[WRITE] "};

/*! \brief Write a header for the log file including some meta-data about the journaller
*/
void Journaller::writeFileHeader(const std::string& appName)
{
	m_appName = appName;
	XsTimeStamp now = XsTimeStamp::now();
	JLWRITE(this, "Journaller logging to " << m_file->filename() << (appName.empty() ? XsString() : XsString(" for ") + appName) << " on " << now.toString());
	//JLWRITE(this, "Current log level is " << gLogLevelString[m_level]);
}

/*! \brief Write a log message to the file if \a level is at least equal to the current log level
	\param level The log level to use
	\param msg The message to log
	\details This function decorates \a msg with the time and log level and appends a newline
*/
void Journaller::log(JournalLogLevel level, const std::string& msg)
{
#ifdef ANDROID
	__android_log_write(level, tag().c_str(), msg.c_str());
#else
	if (level < m_level && level < m_debugLevel)
		return;

	m_threader->setLineLevel(threadId(), level);
	writeTime();
#if JOURNALLER_WITH_THREAD_SUPPORT
	writeThread();
#endif
	writeTag();
	writeLevel(level);
	writeMessage(msg);
	writeMessage(std::string("\n"));
	if (level >= m_flushLevel)
		flush();
#endif
}

/*! \brief Write the current time to the file */
void Journaller::writeTime()
{
	XsTimeStamp ts = XsTimeStamp::now();
	if (!m_useDateTime)
	{
		// when using timestamp format we use UTC time!
		char timebuf[32];
		sprintf(timebuf, "%10" PRINTF_INT64_MODIFIER "d.%03d ", ts.secondTime(), (int) ts.milliSecondPart());
		writeMessage(timebuf);
	}
	else
	{
		// when using date time format we use LOCAL time!
		writeMessage(ts.utcToLocalTime().toString().toStdString());
	}
}

/*! \brief Write the current time to the file */
void Journaller::writeThread()
{
	char buf[32];
#ifdef __GNUC__
	sprintf(buf, "<%08X> ", (unsigned int) threadId());
#else
	sprintf(buf, "<%04X> ", (unsigned int) threadId());
#endif
	writeMessage(buf);
}

/*! \brief Write the tag to the file */
void Journaller::writeTag()
{
	std::string tmp = tag();
	if (!tmp.empty())
		writeMessage(tmp);
}

/*! \brief Write the supplied log \a level to the file
	\param level The log level to set
	\note This does not actually CHANGE the log level of the line to the supplied value
*/
void Journaller::writeLevel(JournalLogLevel level)
{
	writeMessage(std::string(gLogLevelString[level]));
}

/*! \brief Write \a msg to the file without decoration or added newline
	\param msg The message to write
	\details This function is also used to do the actual writing. It will only write the data to the
	outputs when the end of \a msg is a newline character, otherwise it will queue \a msg to be
	written later. If \a msg is empty, the currently queued data is written immediately
*/
void Journaller::writeMessage(const std::string& msg)
{
	if (msg.empty())
	{
		flushLine();
		return;
	}

	m_threader->line(threadId()).append(msg);
	char eol = *msg.rbegin();
	if (eol == '\n' || eol == '\r')
		flushLine();
}

/*! \brief Flush any data currently queued for logging to the file buffer
*/
void Journaller::flushLine()
{
	int thread = threadId();
	std::string& line = m_threader->line(thread);
	JournalLogLevel lineLevel = m_threader->lineLevel(thread);
	if (!line.empty())
	{
		if (lineLevel >= m_level)
			m_threader->writeLine(thread, m_file.get());
		if (lineLevel > m_debugLevel)
			m_threader->writeLine(thread, nullptr);
		line.clear();
	}
}

/*! \brief Flush any data to disk */
void Journaller::flush()
{
	if (m_file)
	{
		flushLine();
		m_file->flush();
	}
}

/*! \brief Set level threshold for automatically flushing lines to disk. */
void Journaller::setFlushLevel(JournalLogLevel level, bool writeLogLine)
{
	m_flushLevel = level;
	if (writeLogLine)
		JLGENERIC(this, JLL_Write, "Flush level switched to " << gLogLevelString[m_flushLevel]);
}

/*! \brief Write the current callstack to the file if \a level is at least equal to the current log level
*/
void Journaller::writeCallstack(JournalLogLevel level)
{
	if (level < m_level)
		return;

	JLGENERIC(this, level, "************ Dump Begin ************");
	JournalStackWalker sw(this);
	sw.ShowCallstack();
	JLGENERIC(this, level, "************* Dump End *************");
}

/*! \brief Set the log level for logging to file
	\param level The log level to use
	\param writeLogLine If set to true then it writes a log line
	\details Any log requests of a lower level will not be done
*/
void Journaller::setLogLevel(JournalLogLevel level, bool writeLogLine)
{
	m_level = level;
	if (writeLogLine)
		JLGENERIC(this, JLL_Write, "Log level set to " << gLogLevelString[level]);
}

/*! \brief Set the log level for logging to debug output
	\param level The log level to use
	\param writeLogLine If set to true then it writes a log line
	\details Any log requests of a lower level will not be done
*/
void Journaller::setDebugLevel(JournalLogLevel level, bool writeLogLine)
{
	m_debugLevel = level;
	if (writeLogLine)
		JLGENERIC(this, JLL_Write, "Debugger output log level set to " << gLogLevelString[level]);
}

/*! \brief Returns the filename of the used journal file */
const XsString Journaller::filename() const
{
	return m_file->filename();
}

/*! \brief Set a tag to be added before the log level tag in each log file
*/
void Journaller::setTag(const std::string &tag)
{
	m_tag = tag;
}

/*! \returns a tag that will be added before the log level tag in each log file.
	\details On most platforms the default tag is an empty string, on
	Android it defaults to the filename without extension to be able to
	make full use of the logcat backend.

	Override the tag using setTag, or overload this function.
*/
std::string Journaller::tag() const
{
	return m_tag;
}

template <>
std::ostream& operator << (std::ostream& os, JlHexLogger<char> const& hex)
{
	return os << JlHexLogger<int>((int) hex.m_value);
}

/*! \brief Move the log file to the supplied path, keeping current contents intact
	\param pathfile The path of a file to change to
	\param purge If set to true (default) then it clears the log file that we changed to
	\param eraseOld If set to true (default) then the old log file will be removed
*/
void Journaller::moveLogFile(const XsString& pathfile, bool purge, bool eraseOld)
{
	if (m_file && m_file->filename() == pathfile)
		return;

	JournalFile* newFile = new JournalFile(pathfile, purge);

	XsString oldFileName;
	std::unique_ptr<uint8_t[]> buffer;
	XsFilePos sz = 0;
	if (m_file && m_file->xsFile().isOpen())
	{
		oldFileName = m_file->filename();
		JLWRITE(this, "Switching to file " << pathfile);

		XsFile& file = m_file->xsFile();
		sz = file.tell();
		if (sz)
		{
			file.seek(0);
			buffer.reset(new uint8_t[(int)sz]);
			sz = file.read(buffer.get(), 1, sz);
		}
	}

	m_file.reset(newFile);
	if (buffer && sz)
		m_file->xsFile().write(buffer.get(), 1, sz);
	else
		writeFileHeader(m_appName);

	if (!oldFileName.empty())
	{
		if (eraseOld)
			_unlink(oldFileName.c_str());
		JLDEBUG(this, "Switched from " << oldFileName << " to " << pathfile);
	}
	else
		JLDEBUG(this, "Switched to " << pathfile);
}

/*! \brief Copy the contents of the current log file to \a target
	\details After copying the contents, the current log file is closed.
	\param target The destination Journaller to write to
	\param eraseOld When true (default), the old log file is deleted
*/
void Journaller::moveLogs(Journaller* target, bool eraseOld)
{
	assert(target);
	if (target->filename() == filename())
		return;

	auto newFile = target->m_file;
	XsString oldFileName;
	std::unique_ptr<uint8_t[]> buffer;
	XsFilePos sz = 0;
	if (m_file && m_file->xsFile().isOpen())
	{
		oldFileName = m_file->filename();
		JLDEBUG(target, "************ Moving logs from " << oldFileName << " to " << target->filename());

		XsFile& file = m_file->xsFile();
		sz = file.tell();
		if (sz)
		{
			file.seek(0);
			buffer.reset(new uint8_t[(int)sz]);
			sz = file.read(buffer.get(), 1, sz);
		}
	}

	if (buffer && sz && newFile)
		newFile->xsFile().write(buffer.get(), 1, sz);
	m_file = newFile;

	if (!oldFileName.empty())
	{
		if (eraseOld)
			_unlink(oldFileName.c_str());
		JLDEBUG(target, "************ Moved logs from " << oldFileName);
	}
}

/*! \brief Cleans up any remaining stuff
	\details The function assumes that no new log lines will be created anymore
	\param gj Optional pointer to Journaller pointer to be cleaned up, usually &gJournal should be provided. The pointer that gj points to will be set to nullptr.
	\note JLINSTALLQTDEBUGHANDLER(0) is not done as it would create a dependency on Qt so you need to call that yourself before calling jlTerminate
*/
void jlTerminate(Journaller** gj)
{
	if (gj && *gj)
	{
		delete *gj;
		*gj = nullptr;
	}
}

#if __GNUC__ >= 6
#pragma GCC diagnostic pop
#endif
