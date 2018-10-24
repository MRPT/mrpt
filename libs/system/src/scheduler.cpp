/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "system-precomp.h"  // Precompiled headers

#include <mrpt/system/scheduler.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/config.h>

#ifdef MRPT_OS_WINDOWS
#include <windows.h>
#include <process.h>
#include <tlhelp32.h>
#else
#include <pthread.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/time.h>
#include <ctime>
#include <unistd.h>
#include <utime.h>
#include <cerrno>
#include <csignal>
#include <cstring>  // strerror()
#endif

#include <sys/types.h>
#include <sys/stat.h>
#ifdef MRPT_OS_APPLE
#include <sys/sysctl.h>
#include <mach/mach_init.h>
#include <mach/thread_act.h>
#endif

#include <iostream>

void mrpt::system::changeCurrentThreadPriority(TThreadPriority priority)
{
#ifdef MRPT_OS_WINDOWS
	// TThreadPriority is defined to agree with numbers expected by Win32 API:
	SetThreadPriority(GetCurrentThread(), priority);
#else
	const pthread_t tid =
#ifdef MRPT_OS_APPLE
		reinterpret_cast<long unsigned int>(pthread_self());
#else
		pthread_self();
#endif

	int ret, policy;
	struct sched_param param
	{
	};

	if (0 != (ret = pthread_getschedparam(tid, &policy, &param)))
	{
		std::cerr
			<< "[mrpt::system::changeThreadPriority] Warning: Failed call to "
			   "pthread_getschedparam (error: `"
			<< strerror(ret) << "`)" << std::endl;
		return;
	}

	policy = SCHED_RR;
	int min_prio = sched_get_priority_min(policy),
		max_prio = sched_get_priority_max(policy);
	if (min_prio < 0) min_prio = 1;  // Just in case of error to calls above (!)
	if (max_prio < 0) max_prio = 99;

	int prio = 0;
	switch (priority)
	{
		case tpLowests:
			prio = min_prio;
			break;
		case tpLower:
			prio = (max_prio + 3 * min_prio) / 4;
			break;
		case tpLow:
			prio = (max_prio + 2 * min_prio) / 3;
			break;
		case tpNormal:
			prio = (max_prio + min_prio) / 2;
			break;
		case tpHigh:
			prio = (2 * max_prio + min_prio) / 3;
			break;
		case tpHigher:
			prio = (3 * max_prio + min_prio) / 4;
			break;
		case tpHighest:
			prio = max_prio;
			break;
	}

	param.sched_priority = prio;
	if (0 != (ret = pthread_setschedparam(tid, policy, &param)))
	{
		std::cerr
			<< "[mrpt::system::changeThreadPriority] Warning: Failed call to "
			   "pthread_setschedparam (error: `"
			<< strerror(ret) << "`)" << std::endl;
		return;
	}
#endif
}

void mrpt::system::changeCurrentProcessPriority(TProcessPriority priority)
{
#ifdef MRPT_OS_WINDOWS
	DWORD dwPri;
	switch (priority)
	{
		case ppIdle:
			dwPri = IDLE_PRIORITY_CLASS;
			break;
		case ppNormal:
			dwPri = NORMAL_PRIORITY_CLASS;
			break;
		case ppHigh:
			dwPri = HIGH_PRIORITY_CLASS;
			break;
		case ppVeryHigh:
			dwPri = REALTIME_PRIORITY_CLASS;
			break;
		default:
			THROW_EXCEPTION("Invalid priority value");
	}
	SetPriorityClass(GetCurrentProcess(), dwPri);
#else
	int nice_val;
	switch (priority)
	{
		case ppIdle:
			nice_val = +19;
			break;
		case ppNormal:
			nice_val = 0;
			break;
		case ppHigh:
			nice_val = -10;
			break;
		case ppVeryHigh:
			nice_val = -20;
			break;
		default:
			THROW_EXCEPTION("Invalid priority value");
	}
	errno = 0;
	const int ret = nice(nice_val);
	if (ret == -1 && errno == EPERM)
	{
		std::cerr << "[mrpt::system::changeCurrentProcessPriority] Error "
					 "calling nice(): Not enough permissions.\n";
	}
#endif
}
