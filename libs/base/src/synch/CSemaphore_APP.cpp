/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
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
    unsigned int    maxCount)
{
  MRPT_START

	m_data.resize(sizeof(semaphore_t));
	semaphore_t * sem = m_data.getAsPtr<semaphore_t>();

	kern_return_t error = semaphore_create(mach_task_self(), sem, SYNC_POLICY_FIFO, initialCount);
	if(error != KERN_SUCCESS)
		THROW_EXCEPTION( format("Creating semaphore raised error: %d", error ) );

  MRPT_END
}

// ######################################################################
CSemaphore::~CSemaphore()
{
  MRPT_START

  if (m_data.alias_count()==1)
  {
	semaphore_t * sem = m_data.getAsPtr<semaphore_t>();
	semaphore_destroy(mach_task_self(), *sem);
  }

  MRPT_END
}

// ######################################################################
bool CSemaphore::waitForSignal( unsigned int timelimit )
{
  MRPT_START

	semaphore_t * sem = m_data.getAsPtr<semaphore_t>();

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

  MRPT_END
}

// ######################################################################
void CSemaphore::release(unsigned int increaseCount )
{
  MRPT_START

	semaphore_t * sem = m_data.getAsPtr<semaphore_t>();
	for(unsigned int i=0; i<increaseCount; ++i)
	{
		kern_return_t result = semaphore_signal(*sem);
		if(result != KERN_SUCCESS)
		THROW_EXCEPTION( format("Increasing count of semaphore raised error: %d", result ) );
	}

  MRPT_END
}

#endif // defined(MRPT_OS_APPLE)
