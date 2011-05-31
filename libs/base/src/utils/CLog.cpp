/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
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
