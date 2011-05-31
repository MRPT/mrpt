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

#include <mrpt/base.h>  // Only for precomp. headers, include all libmrpt-core headers.


#include <mrpt/config.h>
#if defined(MRPT_OS_LINUX) || defined(MRPT_OS_APPLE)

#include <mrpt/synch/CSemaphore.h>
#include <mrpt/utils/CStdOutStream.h>


#include <iostream>
#include <pthread.h>
#include <errno.h>
#include <fcntl.h>  // O_CREAT
#include <semaphore.h>
#include <sys/timeb.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::synch;
using namespace std;


typedef struct
{
	sem_t * semid;
	bool    has_to_free_mem;
} sem_private_struct, *sem_private;


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

	// Reserve memory for my data:
    m_data.resize( sizeof(sem_private_struct) );
    sem_private token = m_data.getAs<sem_private>();

	if (isNamed())
	{
		// Named semaphores assume a Linux kernel 2.6+
		// See: http://linux.die.net/man/3/sem_open

		token->has_to_free_mem = false;  // the "sem_t*" is returned by sem_open()

		// Open it or create if not existing:
		token->semid = sem_open(m_name.c_str(),O_CREAT, 0644 /* permisions */, initialCount );
	}
	else
	{
		// Unnamed semaphore:
		token->has_to_free_mem = true;  // sem_init() requires an already allocated "sem_t"
		token->semid = static_cast<sem_t*>( malloc(sizeof(sem_t)) );

		if (sem_init(token->semid, 0 /*pshared:false*/, initialCount))
			token->semid=SEM_FAILED;
	}


	// On error, launch an exception explaining it:
	if (token->semid==SEM_FAILED)
		THROW_EXCEPTION( format("Creating semaphore (name='%s') raised error: %s",m_name.c_str(),strerror(errno) ) )

	MRPT_END
}

/*---------------------------------------------------------------
						~CSemaphore
---------------------------------------------------------------*/
CSemaphore::~CSemaphore()
{
	if (m_data.alias_count()==1)
	{
		sem_private token = m_data.getAs<sem_private>();

		sem_destroy(token->semid);

		if (token->has_to_free_mem)
			free(token->semid);
	}
}

/*---------------------------------------------------------------
Blocks until the count of the semaphore to be non-zero.
\param timeout_ms The timeout in milliseconds, or set to zero to wait indefinidely.
\return true if the semaphore has been signaled, false on timeout or any other error.
---------------------------------------------------------------*/
bool CSemaphore::waitForSignal( unsigned int timelimit )
{
	MRPT_START

    sem_private token = m_data.getAs<sem_private>();

	// Prepare the "tm" struct with the absolute timeout timestamp:
    struct timeb tp;

    const long sec = timelimit / 1000;
    const long millisec = timelimit % 1000;
    ftime( &tp );
    tp.time += sec;
    tp.millitm += millisec;
    if( tp.millitm > 999 )
    {
        tp.millitm -= 1000;
        tp.time++;
    }

    struct timespec tm;
    tm.tv_sec = tp.time;
    tm.tv_nsec = tp.millitm * 1000000 ;

	int rc = timelimit==0 ?
		// No timeout
		sem_wait( token->semid )
		:
		// We have a timeout:
		sem_timedwait( token->semid, &tm );

	// If there's an error != than a timeout, dump to stderr:
	if (rc!=0 && errno!=ETIMEDOUT)
		std::cerr << format("[CSemaphore::waitForSignal] In semaphore named '%s', error: %s\n", m_name.c_str(),strerror(errno) );

	return rc==0; // true: all ok.

	MRPT_END
}

/*---------------------------------------------------------------
	Increments the count of the semaphore by a given amount.
---------------------------------------------------------------*/
void CSemaphore::release(unsigned int increaseCount )
{
	MRPT_START

    sem_private token = m_data.getAs<sem_private>();

    for (unsigned int i=0;i<increaseCount;i++)
    	if (sem_post(token->semid))
			THROW_EXCEPTION( format("Increasing count of semaphore (name='%s') raised error: %s",m_name.c_str(),strerror(errno) ) )

	MRPT_END
}


#endif // Linux
