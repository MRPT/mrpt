
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

#include "journalthreader.h"
#include "journalfile.h"

#ifndef XSENS_WINDOWS
static void OutputDebugStringA(const char *msg)
{
	(void)fprintf(stderr, "%s", msg);
}
#endif

/*! \class JournalThreader
	\brief Manages threaded writes for the Journaller objects
*/

/*! \brief Constructor, sets up the necessary structures for threaded logging */
JournalThreader::JournalThreader()
{
}

/*! \brief Destructor, flushes all remaining data
*/
JournalThreader::~JournalThreader()
{
}

/*! \brief Flush all available non-empty lines to the supplied file, clearing lines as they are flushed
	\param file The file to write to. Supply nullptr to write to the debug output
*/
void JournalThreader::flushAll(JournalFile* file)
{
	while(true)
	{
		ThreadLine& line = nextLine();
		if (line.m_line.empty())
			return;

		if (file)
			*file << line.m_line;
		else
			OutputDebugStringA(line.m_line.c_str());
		line.m_line.clear();
	}
}

/*! \brief Write the line for thread \a thread to file \a file
	\param thread The thread id associated with the line
	\param file The file to write to. Supply nullptr to write to the debug output
*/
void JournalThreader::writeLine(int thread, JournalFile* file)
{
	ThreadLine& line = threadLine(thread);
	if (line.m_line.empty())
		return;

	if (file)
		*file << line.m_line;
	else
		OutputDebugStringA(line.m_line.c_str());
}

/*! \brief Set the log level of the queued line
	\param thread The thread id associated with the line
	\param level The log level to set
	\returns The old log level
*/
JournalLogLevel JournalThreader::setLineLevel(int thread, JournalLogLevel level)
{
	ThreadLine& line = threadLine(thread);
	auto old = line.m_level;
	line.m_level = level;
	return old;
}

/*! \returns The log level of the queued line
	\param thread The thread id associated with the line
*/
JournalLogLevel JournalThreader::lineLevel(int thread)
{
	ThreadLine& line = threadLine(thread);
	return line.m_level;
}

/*! \returns The reference to the line associated with the given file and thread
	\param thread The thread id associated with the line, ignored when JOURNALLER_WITH_THREAD_SUPPORT == 0
*/
std::string& JournalThreader::line(int thread)
{
	ThreadLine& line = threadLine(thread);
	return line.m_line;
}
