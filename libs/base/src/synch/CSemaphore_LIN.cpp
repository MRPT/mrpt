/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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
#include <semaphore.h>
#include <sys/timeb.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::synch;
using namespace std;


typedef struct
{
	pthread_mutex_t	    mutex;
	pthread_cond_t	    condition;
	int			        semCount;
} sem_private_struct, *sem_private;

/*---------------------------------------------------------------
						CSemaphore
---------------------------------------------------------------*/
CSemaphore::CSemaphore(
    unsigned int    initialCount,
    unsigned int    maxCount,
    const std::string &name )
{
	MRPT_TRY_START;

	if (name.size())    THROW_EXCEPTION("Named semaphores not supported for Linux!");

    // Based on code from:
    //  http://www.ibm.com/developerworks/eserver/library/es-win32linux-sem.html
    m_data.resize( sizeof(sem_private_struct) + 10 );

    sem_private token = m_data.getAs<sem_private>();

    if( pthread_mutex_init(&(token->mutex), NULL) )
    {
    	// m_data memory will be freed automatically
        THROW_EXCEPTION("Error creating semaphore (mutex)!!");
    }

    if( pthread_cond_init(&(token->condition), NULL))
    {
        pthread_mutex_destroy( &( token->mutex) );
    	// m_data memory will be freed automatically
        THROW_EXCEPTION("Error creating semaphore (cond)!!");
    }

    token->semCount = initialCount;


	MRPT_TRY_END;
}

/*---------------------------------------------------------------
						~CSemaphore
---------------------------------------------------------------*/
CSemaphore::~CSemaphore()
{
	if (m_data.alias_count()==1)
	{
		sem_private token = m_data.getAs<sem_private>();

		pthread_mutex_destroy(&(token->mutex));
		pthread_cond_destroy(&(token->condition));
	}
}

/*---------------------------------------------------------------
Blocks until the count of the semaphore to be non-zero.
\param timeout_ms The timeout in milliseconds, or set to zero to wait indefinidely.
\return true if the semaphore has been signaled, false on timeout or any other error.
---------------------------------------------------------------*/
bool CSemaphore::waitForSignal( unsigned int timelimit )
{
	MRPT_TRY_START;

    sem_private token = m_data.getAs<sem_private>();
    int rc;
    struct timespec tm;
    struct timeb tp;
    long sec, millisec;

    if ( (rc = pthread_mutex_lock(&(token->mutex))) )
    {
        cerr << "[CSemaphore::waitForSignal] Error in pthread_mutex_lock: "<< rc << endl;
        return false;
    }

    sec = timelimit / 1000;
    millisec = timelimit % 1000;
    ftime( &tp );
    tp.time += sec;
    tp.millitm += millisec;
    if( tp.millitm > 999 )
    {
        tp.millitm -= 1000;
        tp.time++;
    }
    tm.tv_sec = tp.time;
    tm.tv_nsec = tp.millitm * 1000000 ;

    while (token->semCount <= 0)
    {
        if ( !timelimit ) // No timeout
        {
            rc = pthread_cond_wait(&(token->condition), &(token->mutex) );
        }
        else
        {   // We have a timeout:
            rc = pthread_cond_timedwait(&(token->condition), &(token->mutex), &tm);
        }

        if (rc && (errno != EINTR) )
            break;
    }
    if ( rc )
    {
        if ( pthread_mutex_unlock(&(token->mutex)) )
        {
            cerr << "[CSemaphore::waitForSignal] Error in pthread_mutex_unlock: "<< rc << endl;
            return false; //RC_SEM_WAIT_ERROR;
        }

        if ( rc == ETIMEDOUT) /* we have a time out */
        {
            //cerr << "[CSemaphore::waitForSignal] TIMEOUT "<< endl;
            return false;
        }

        return false;
    }
    token->semCount--;

    if ( (rc = pthread_mutex_unlock(&(token->mutex))) )
    {
        cerr << "[CSemaphore::waitForSignal] Error in pthread_mutex_unlock: " << rc << endl;
        return false; //RC_SEM_WAIT_ERROR;
    }

    // Signaled!
    return true;

	MRPT_TRY_END;
}

/*---------------------------------------------------------------
	Increments the count of the semaphore by a given amount.
---------------------------------------------------------------*/
void CSemaphore::release(unsigned int increaseCount )
{
	MRPT_TRY_START;

    sem_private token = m_data.getAs<sem_private>();

    if ( pthread_mutex_lock(&(token->mutex)) )
        THROW_EXCEPTION("Error increasing semaphore count! (mutex_lock)");

    token->semCount +=increaseCount;

    if ( pthread_mutex_unlock(&(token->mutex)))
        THROW_EXCEPTION("Error increasing semaphore count! (mutex_unlock)");

    if ( pthread_cond_signal(&(token->condition)))
        THROW_EXCEPTION("Error increasing semaphore count! (cond_signal)");

	MRPT_TRY_END;
}



#endif // Linux
