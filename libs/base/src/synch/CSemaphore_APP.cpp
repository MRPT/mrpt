/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Only for precomp. headers, include all libmrpt-core headers.


#include <mrpt/config.h>
#if defined(MRPT_OS_APPLE)

#include <mrpt/synch/CSemaphore.h>
#include <mrpt/utils/CStdOutStream.h>
#include <mrpt/system/threads.h>

#include <mach/mach_init.h>
#include <mach/task.h>
#include <mach/semaphore.h>

#include <iostream>
#include <pthread.h>
#include <errno.h>
#include <fcntl.h>  // O_CREAT
#include <semaphore.h>
#include <sys/timeb.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::synch;

typedef struct
{
  sem_t * semid;
} sem_private_struct, *sem_private;

// ######################################################################
CSemaphore::CSemaphore(
    unsigned int    initialCount,
    unsigned int    maxCount,
    const std::string &name )
    :
    m_name(name)
{
  MRPT_START

  if (isNamed())
  {
    m_data.resize( sizeof(sem_private_struct) );
    sem_private token = m_data.getAs<sem_private>();

    // Open it or create if not existing:
    token->semid = sem_open(m_name.c_str(), O_CREAT, 0644 /* permisions */, initialCount );

    if (token->semid==SEM_FAILED)
      THROW_EXCEPTION( format("Creating semaphore (name='%s') raised error: %s",m_name.c_str(),strerror(errno) ) )
  }
  else
  {
    m_data.resize(sizeof(semaphore_t));
    semaphore_t * sem = m_data.getAs<semaphore_t *>();

    kern_return_t error = semaphore_create(mach_task_self(), sem, SYNC_POLICY_FIFO, initialCount);
    if(error != KERN_SUCCESS)
      THROW_EXCEPTION( format("Creating unnamed semaphore raised error: %d", error ) );
  }

  MRPT_END
}

// ######################################################################
CSemaphore::~CSemaphore()
{
  MRPT_START

  if (m_data.alias_count()==1)
  {
    if(isNamed())
    {
      sem_private token = m_data.getAs<sem_private>();
      sem_close(token->semid);
    }
    else
    {
      semaphore_t * sem = m_data.getAs<semaphore_t *>();
      semaphore_destroy(mach_task_self(), *sem);
    }
  }

  MRPT_END
}

// ######################################################################
bool CSemaphore::waitForSignal( unsigned int timelimit )
{
  MRPT_START

  if(isNamed())
  {
    struct timeb endtime;

    const long sec = timelimit / 1000;
    const long millisec = timelimit % 1000;
    ftime( &endtime );
    endtime.time += sec;
    endtime.millitm += millisec;
    if( endtime.millitm > 999 )
    {
        endtime.millitm -= 1000;
        endtime.time++;
    }

    sem_private token = m_data.getAs<sem_private>();

    struct timeb nowtime;
    ftime( &nowtime );
    int rc;
    // Mac version: we don't have sem_timedwait()
    if(timelimit==0)
    {
      // No timeout
      rc = sem_wait(token->semid);
      return (rc == 0); // true: all ok.
    }

    do
    {
      rc = sem_trywait(token->semid);
      mrpt::system::sleep(1);
      ftime( &nowtime );
    }
    while(rc != 0 && (endtime.time > nowtime.time && endtime.millitm > nowtime.millitm));

    return (rc == 0); // true: all ok.
  }
  else
  {
    semaphore_t * sem = m_data.getAs<semaphore_t *>();

    if(timelimit == 0)
    {
      kern_return_t result = semaphore_wait(*sem);

      return (result == KERN_SUCCESS);
    }
    else
    {
      mach_timespec_t ts;
      ts.tv_sec = timelimit / 1000;
      ts.tv_nsec = (timelimit % 1000) * 1000000;

      kern_return_t result = semaphore_timedwait(*sem, ts);

      return (result == KERN_SUCCESS);
    }
  }

  MRPT_END
}

// ######################################################################
void CSemaphore::release(unsigned int increaseCount )
{
  MRPT_START

  if(isNamed())
  {
    sem_private token = m_data.getAs<sem_private>();

    for (unsigned int i=0;i<increaseCount;i++)
      if (sem_post(token->semid))
      THROW_EXCEPTION( format("Increasing count of semaphore (name='%s') raised error: %s",m_name.c_str(),strerror(errno) ) )
  }
  else
  {
    semaphore_t * sem = m_data.getAs<semaphore_t *>();
    for(unsigned int i=0; i<increaseCount; ++i)
    {
      kern_return_t result = semaphore_signal(*sem);
      if(result != KERN_SUCCESS)
        THROW_EXCEPTION( format("Increasing count of unnamed semaphore raised error: %d", result ) );
    }
  }

  MRPT_END
}

#endif // defined(MRPT_OS_APPLE)
