/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

/***************************************************************
 * Name:      Log.cpp
 * Purpose:   Code for the Log class
 * Author:    Vicente Arévalo (varevalo@ctima.uma.es)
 * Created:   2009-09-23
 * Copyright: mapir (http://babel.isa.uma.es/mapir)
 * License:
 **************************************************************/

#include <mrpt/base.h>  // Precompiled headers

#include <mrpt/utils/CLog.h>
#include <mrpt/system/datetime.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::system;


CLog::CLog() : last(0)
{
}

CLog::~CLog()
{
}

/** push a message.
*/
void CLog::pushMessages(std::string message)
{
	mrpt::synch::CCriticalSectionLocker cs( &semaphore );

	// get timestamp
	TTimeStamp time = mrpt::system::now();
	std::string timestamp = timeLocalToString(time);

	// add the message
	add(timestamp + ":\t" + message);
}

/** push a list of messages.
*/
void CLog::pushMessages(mrpt::utils::CStringList messages)
{
	mrpt::synch::CCriticalSectionLocker cs( &semaphore );

	std::string aux;
	for( size_t i = 0; i < messages.size(); i++)
	{
		// get timestamp
		TTimeStamp time = mrpt::system::now();
		std::string timestamp = timeLocalToString(time);

		messages.get(i, aux);

		// add the message
		add(timestamp + ":\t" + aux);
	}
}

/** pop the current unpublished message (clear the content of "message").
*/
void CLog::popMessages(std::string& message)
{
	mrpt::synch::CCriticalSectionLocker cs( &semaphore );

	get(last++, message);
}

/** pop all unpublished messages (clear the content of "messages").
*/
void CLog::popMessages(mrpt::utils::CStringList& messages)
{
	mrpt::synch::CCriticalSectionLocker cs( &semaphore );

	messages.clear();

	std::string aux;
	for(size_t i = last; i < size(); i++, last++)
	{
		get(i, aux);
		messages.add(aux);
	}
}

/** get messages from "begin" to "end" (clear the content of "messages").
*/
void CLog::getMessages(size_t begin, size_t end, mrpt::utils::CStringList& messages)
{
	mrpt::synch::CCriticalSectionLocker cs( &semaphore );

	messages.clear();

	std::string aux;
	for(size_t i = begin; i <= end; i++)
	{
		get(i, aux);
		messages.add(aux);
	}
}

/** save the current log.
*/
void CLog::saveLog(std::string name)
{
	mrpt::synch::CCriticalSectionLocker cs( &semaphore );

	saveToFile( name );
}

/** load a log (clear the previous content).
*/
void CLog::loadLog(std::string name)
{
	mrpt::synch::CCriticalSectionLocker cs( &semaphore );

	loadFromFile( name );

	last = 0;
}

/** clear the log content
*/
void CLog::clearLog()
{
	mrpt::synch::CCriticalSectionLocker cs( &semaphore );

	clear();

	last = 0;
}

/** change the last unpublished message. This function should not be used directly !!!
*/
void CLog::setLastMessageIndex(size_t index)
{
	mrpt::synch::CCriticalSectionLocker cs( &semaphore );
	last = index;
}

/** get the current unpublished message index.
*/
size_t CLog::getLastMessageIndex()
{
	mrpt::synch::CCriticalSectionLocker cs( &semaphore );
	return last;
}
