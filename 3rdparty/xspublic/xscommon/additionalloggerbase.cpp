
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

#include "additionalloggerbase.h"

/*! \brief Constructs an AdditionalLoggerBase
	\param[in] initialLogLevel The initial log level to use as log level and debug log level
*/
AdditionalLoggerBase::AdditionalLoggerBase(JournalLogLevel initialLogLevel)
	: m_level(initialLogLevel), m_debugLevel(initialLogLevel)
{
}

/*! \copydoc AbstractAdditionalLogger::logLevel() const
*/
JournalLogLevel AdditionalLoggerBase::logLevel() const
{
	return m_level;
}

/*! \copydoc AbstractAdditionalLogger::debugLevel
*/
JournalLogLevel AdditionalLoggerBase::debugLevel() const
{
	return m_debugLevel;
}

/*! \copydoc AbstractAdditionalLogger::setLogLevel
*/
void AdditionalLoggerBase::setLogLevel(JournalLogLevel level)
{
	if (level != m_level)
	{
		JournalLogLevel oldLevel = m_level;
		m_level = level;
		onLogLevelChanged(m_level, oldLevel);
	}
}

/*! \copydoc AbstractAdditionalLogger::setDebugLevel
*/
void AdditionalLoggerBase::setDebugLevel(JournalLogLevel level)
{
	if (level != m_debugLevel)
	{
		JournalLogLevel oldLevel = m_debugLevel;
		m_debugLevel = level;
		onDebugLevelChanged(m_debugLevel, oldLevel);
	}
}

/*! \brief Called when the log level changes
	This can be overridden in derived classes
*/
void AdditionalLoggerBase::onLogLevelChanged(JournalLogLevel newLevel, JournalLogLevel oldLevel)
{
	(void)newLevel;
	(void)oldLevel;
}

/*! \brief Called when the debug log level changes
	This can be overridden in derived classes
*/
void AdditionalLoggerBase::onDebugLevelChanged(JournalLogLevel newLevel, JournalLogLevel oldLevel)
{
	(void)newLevel;
	(void)oldLevel;
}

/*! \copydoc AbstractAdditionalLogger::logLevel
*/
bool AdditionalLoggerBase::logLevel(JournalLogLevel level) const
{
	return level >= m_level || level >= m_debugLevel;
}

/*! \copydoc AbstractAdditionalLogger::logNoDecoration
	The AdditionalLoggerBase simply forwards this call to the normal log method but this method can be overridden in derived classes
*/
void AdditionalLoggerBase::logNoDecoration(JournalLogLevel level, char const * file, int line, char const * function, std::string const & msg)
{
	log(level, file, line, function, msg);
}
