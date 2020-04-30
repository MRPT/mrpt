
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

#ifndef JOURNALTHREADER_H
#define JOURNALTHREADER_H

#include "xscommon_config.h"
#include <string>
#include "journalloglevel.h"
#if JOURNALLER_WITH_THREAD_SUPPORT
#include <map>
#include "xsens_mutex.h"
#endif

class FileInfo;
class JournalFile;

class JournalThreader {
public:
	JournalThreader();
	~JournalThreader();

	void flushAll(JournalFile* file);
	void writeLine(int thread, JournalFile* file);
	JournalLogLevel setLineLevel(int thread, JournalLogLevel level);
	JournalLogLevel lineLevel(int thread);
	std::string& line(int thread);

private:
	/*! \brief Storage for logging queue of a specific thread
	*/
	class ThreadLine {
	public:
		std::string m_line;			//!< The contained text for this log line
		JournalLogLevel m_level;	//!< The level of this log line

		/*! \brief Constructor, creates an empty line at WRITE log level */
		ThreadLine() : m_level(JLL_Write) {}
	};

#if JOURNALLER_WITH_THREAD_SUPPORT
	std::map<int, ThreadLine> m_map;		//!< The contained lines, one for each thread
	mutable xsens::Mutex m_mutex;			//!< A mutex guarding access to the map (not to the lines)

	/*! \brief Get the line object for the supplied thread, typically for the current thread */
	inline ThreadLine& threadLine(int thread)
	{
		xsens::Lock lock(&m_mutex);
		return m_map[thread];
	}

	/*! \brief Get the next non-empty line object from the map of all threads, returns an empty line object if no non-empty objects were available */
	inline ThreadLine& nextLine()
	{
		xsens::Lock lock(&m_mutex);
		for (auto it = m_map.begin(); it != m_map.end(); ++it)
			if (!it->second.m_line.empty())
				return it->second;
		return threadLine(0);
	}

#else

	ThreadLine m_line;			//!< The contained line, in single-threaded mode only a single line is needed per journaller
	/*! \brief Get the line object */
	inline ThreadLine& threadLine(int)
	{
		return m_line;
	}

	/*! \brief Get the line object */
	inline ThreadLine& nextLine()
	{
		return m_line;
	}
#endif
};

#endif
