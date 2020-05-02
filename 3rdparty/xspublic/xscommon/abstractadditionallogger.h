
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

#ifndef ABSTRACTADDITIONALLOGGER_H
#define ABSTRACTADDITIONALLOGGER_H

#include "journalloglevel.h"
#include <string>

/*! \class AbstractAdditionalLogger
	Interface for all additional loggers that can be attached to the Journaller
*/
class AbstractAdditionalLogger {
public:
	/*! \brief Destructor */
	virtual ~AbstractAdditionalLogger() {}

	/*! \returns true if loglevel \a level should be logged, false otherwise
		\param[in] level The log level to check
	*/
	virtual bool logLevel(JournalLogLevel level) const = 0;

	/*! \returns the current log level
	*/
	virtual JournalLogLevel logLevel() const = 0;

	/*! \returns the current debug log level
	*/
	virtual JournalLogLevel debugLevel() const = 0;

	/*! \brief Set the current log level
		\param[in] level The new log level
	*/
	virtual void setLogLevel(JournalLogLevel level) = 0;

	/*! \brief Set the current debug log level
		\param[in] level The new debug log level
	*/
	virtual void setDebugLevel(JournalLogLevel level) = 0;

	/*! \brief Write line to the log
		\param[in] level The log level
		\param[in] file The name of the file from which the logging originates
		\param[in] line The line number from which the logging originates
		\param[in] function The name of the function from which the logging originates
		\param[in] msg The actual log message
	*/
	virtual void log(JournalLogLevel level, char const * file, int line, char const * function, std::string const & msg) = 0;

	/*! \brief Write line to the log without decoration (timestamp, thread)
		\param[in] level The log level
		\param[in] file The name of the file from which the logging originates
		\param[in] line The line number from which the logging originates
		\param[in] function The name of the function from which the logging originates
		\param[in] msg The actual log message
	*/
	virtual void logNoDecoration(JournalLogLevel level, char const * file, int line, char const * function, std::string const & msg) = 0;
};

#endif
