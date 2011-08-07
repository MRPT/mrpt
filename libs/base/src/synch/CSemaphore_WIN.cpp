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

#include <mrpt/base.h>  // Precompiled headers



#include <mrpt/config.h>

#ifdef MRPT_OS_WINDOWS

#include <windows.h>

#include <mrpt/synch/CSemaphore.h>
#include <mrpt/utils/CStream.h>

using namespace mrpt;
using namespace mrpt::synch;

/*---------------------------------------------------------------
						CSemaphore
---------------------------------------------------------------*/
CSemaphore::CSemaphore(
    unsigned int    initialCount,
    unsigned int    maxCount,
    const std::string &name )
    :
    m_name(name)
{
	MRPT_START

	HANDLE hSem = CreateSemaphoreA(
		NULL,			// pointer to security attributes
		initialCount,	// initial count
		maxCount,		// maximum count
		name.size()==0 ? NULL : name.c_str() );

	if (!hSem)	THROW_EXCEPTION("Error creating semaphore!");

	m_data.resize( sizeof(HANDLE) );

	* m_data.getAs<HANDLE*>() = hSem;

	MRPT_END
}

/*---------------------------------------------------------------
						~CSemaphore
---------------------------------------------------------------*/
CSemaphore::~CSemaphore()
{
	if (m_data.alias_count()==1)
	{
		CloseHandle( * m_data.getAs<HANDLE*>() );
	}
}

/*---------------------------------------------------------------
Blocks until the count of the semaphore to be non-zero.
\param timeout_ms The timeout in milliseconds, or set to zero to wait indefinidely.
\return true if the semaphore has been signaled, false on timeout or any other error.
---------------------------------------------------------------*/
bool CSemaphore::waitForSignal( unsigned int timeout_ms )
{
	MRPT_START

	DWORD tim = (timeout_ms==0) ? INFINITE : timeout_ms;
	DWORD ret = WaitForSingleObject( * m_data.getAs<HANDLE*>(), tim );

	return (ret==WAIT_OBJECT_0);

	MRPT_END
}

/*---------------------------------------------------------------
	Increments the count of the semaphore by a given amount.
---------------------------------------------------------------*/
void CSemaphore::release(unsigned int increaseCount )
{
	MRPT_START

	if (!ReleaseSemaphore(
		*m_data.getAs<HANDLE*>(),		// handle of the semaphore object
		increaseCount,		// amount to add to current count
		NULL ))				// address of previous count
			THROW_EXCEPTION("Error increasing semaphore count!");

	MRPT_END
}


#endif // Windows
