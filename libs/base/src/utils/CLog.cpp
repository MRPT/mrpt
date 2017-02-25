/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/***************************************************************
 * Name:      Log.cpp
 * Purpose:   Code for the Log class
 * Author:    Vicente Arévalo (varevalo@ctima.uma.es)
 * Created:   2009-09-23
 * Copyright: mapir (http://babel.isa.uma.es/mapir)
 * License:
 **************************************************************/

#include "base-precomp.h"  // Precompiled headers

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
