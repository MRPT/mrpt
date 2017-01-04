/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/system/threads.h>
#include <mrpt/system/string_utils.h>

#include <mrpt/utils/CTicTac.h>
#include <mrpt/synch/CSemaphore.h>

#ifdef MRPT_OS_WINDOWS
    #include <conio.h>
	#include <windows.h>
	#include <process.h>
	#include <tlhelp32.h>
	#include <sys/utime.h>
	#include <io.h>
	#include <direct.h>
#else
    #include <pthread.h>
    #include <termios.h>
    #include <unistd.h>
    #include <sys/select.h>
	#include <sys/time.h>
    #include <time.h>
	#include <unistd.h>
	#include <utime.h>
	#include <errno.h>
	#include <signal.h>
	#include <string.h> // strerror()
#endif

#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>
#ifdef MRPT_OS_APPLE
	#include <sys/sysctl.h>
	#include <mach/mach_init.h>
	#include <mach/thread_act.h>
#endif

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace std;

/*---------------------------------------------------------------
						sleep
---------------------------------------------------------------*/
void mrpt::system::sleep( int time_ms ) MRPT_NO_THROWS
{
#ifdef MRPT_OS_WINDOWS
	Sleep( time_ms );
#else
	// We will wake up on signals: Assure the desired time has passed:
	CTicTac tictac;
	tictac.Tic();
	int timeLeft_ms = time_ms - (int)(tictac.Tac()*1000);
	while ( timeLeft_ms>0 )
	{
		usleep( timeLeft_ms * 1000 );
		timeLeft_ms = time_ms - (int)(tictac.Tac()*1000);
	}
#endif
}

/*---------------------------------------------------------------
						createThread
---------------------------------------------------------------*/
namespace mrpt
{
	namespace system
	{
		struct TAuxThreadLaucher
		{
	#ifdef MRPT_OS_WINDOWS
			TAuxThreadLaucher() : ptrFunc(NULL),param(NULL),win_sem(0,10)
			{
			}
	#else
			TAuxThreadLaucher() : ptrFunc(NULL),param(NULL) { };
	#endif
			void    (*ptrFunc) (void *);
			void    *param;

	#ifdef MRPT_OS_WINDOWS
			// These for windows only:
			unsigned long		myWindowsId;
			synch::CSemaphore	win_sem;
	#endif
		};

		void *auxiliary_thread_launcher_LIN( void *param )
		{
			try
			{
			TAuxThreadLaucher   *d = reinterpret_cast<TAuxThreadLaucher*>(param);

			TAuxThreadLaucher	localCopy = *d;

	#ifdef MRPT_OS_WINDOWS
			// Signal that the thread has started:
			d->myWindowsId = (unsigned long)GetCurrentThreadId();
			d->win_sem.release();
			// Our parent thread will release the memory of "param".
	#else
			// LINUX: We have to free here this memory:
			delete d;
			d = NULL;
	#endif

			// Now start the user code:
			localCopy.ptrFunc( localCopy.param );
			}
			catch(std::exception &e)
			{
				std::cout << "Exception in [auxiliary_thread_launcher_LIN/WIN]!!!:\n" << e.what();
			}
			catch(...)
			{
				std::cout << "Untyped exception in [auxiliary_thread_launcher_LIN/WIN]!!!\n";
			}
			return NULL;
		}

		void auxiliary_thread_launcher_WIN( void *param )
		{
			auxiliary_thread_launcher_LIN(param);
		}
	} // end namespace
} // end namespace


mrpt::system::TThreadHandle mrpt::system::detail::createThreadImpl(
    void       ( *func )( void * ),
    void       *param
    )
{
	MRPT_START

    TAuxThreadLaucher   *auxData=new TAuxThreadLaucher();
    auxData->ptrFunc = func;
    auxData->param = param;

#ifdef MRPT_OS_WINDOWS
	TThreadHandle		threadHandle;

	HANDLE h= (HANDLE)_beginthread( auxiliary_thread_launcher_WIN,0, auxData);
	if (h== ((HANDLE) -1))
	{
		delete auxData;
		THROW_EXCEPTION("Error creating new thread");
	}

	threadHandle.hThread = h;

	// Wait until the thread starts so we know its ID:
	auxData->win_sem.waitForSignal();
	threadHandle.idThread = auxData->myWindowsId;

	delete auxData; auxData = NULL;

	return threadHandle;

#else
	TThreadHandle		threadHandle;

    pthread_t   newThreadId;
    int iRet = pthread_create( &newThreadId,NULL,auxiliary_thread_launcher_LIN,auxData);
    ASSERT_(iRet==0);

	threadHandle.idThread = (unsigned long)newThreadId;
	return threadHandle;
#endif

	MRPT_END
}


/*---------------------------------------------------------------
						joinThread
---------------------------------------------------------------*/
void mrpt::system::joinThread( const TThreadHandle &threadHandle )
{
	if (threadHandle.isClear()) return;
#ifdef MRPT_OS_WINDOWS
	int prio = GetThreadPriority((HANDLE) threadHandle.hThread);
	if (THREAD_PRIORITY_ERROR_RETURN==prio)
		return; // It seems this is not a running thread...

	DWORD ret = WaitForSingleObject( (HANDLE) threadHandle.hThread , INFINITE );
	if (ret!=WAIT_OBJECT_0)
		cerr << "[mrpt::system::joinThread] Error waiting for thread completion!" << endl;
#elif defined(MRPT_OS_APPLE)
	pthread_join(reinterpret_cast<pthread_t>(threadHandle.idThread), NULL);
#else
	pthread_join(threadHandle.idThread, NULL);
#endif
}

/*---------------------------------------------------------------
						getCurrentThreadId
---------------------------------------------------------------*/
unsigned long mrpt::system::getCurrentThreadId() MRPT_NO_THROWS
{
#ifdef MRPT_OS_WINDOWS
	return GetCurrentThreadId();

/* Jerome Monceaux 2011/03/08: bilock@gmail.com
 * The next precompilation directive didn't compile under osx
 * added defined(MRPT_OS_APPLE) solved the probleme
 */
//#elif MRPT_OS_APPLE
#elif defined(MRPT_OS_APPLE)
	return reinterpret_cast<unsigned long>(pthread_self());
#else
	return pthread_self();
#endif
}

/*---------------------------------------------------------------
						getCurrentThreadHandle
---------------------------------------------------------------*/
TThreadHandle mrpt::system::getCurrentThreadHandle() MRPT_NO_THROWS
{
	TThreadHandle h;
#ifdef MRPT_OS_WINDOWS
	// Win32:
	h.hThread = GetCurrentThread();
	h.idThread = GetCurrentThreadId();
#elif defined(MRPT_OS_APPLE)
	h.idThread = reinterpret_cast<long unsigned int>(pthread_self());
#else
	// pthreads:
	h.idThread = pthread_self();
#endif
	return h;
}

/*---------------------------------------------------------------
					changeThreadPriority
---------------------------------------------------------------*/
void BASE_IMPEXP mrpt::system::changeThreadPriority(
	const TThreadHandle &threadHandle,
	TThreadPriority priority )
{
#ifdef MRPT_OS_WINDOWS
	// TThreadPriority is defined to agree with numbers expected by Win32 API:
	SetThreadPriority( threadHandle.hThread, priority);
#else

	const pthread_t tid =
	#ifdef MRPT_OS_APPLE
		reinterpret_cast<pthread_t>(threadHandle.idThread);
	#else
		threadHandle.idThread;
	#endif

	int ret, policy;
	struct sched_param param;

	if (0!=(ret=pthread_getschedparam(tid,&policy,&param))) {
		cerr << "[mrpt::system::changeThreadPriority] Warning: Failed call to pthread_getschedparam (error: `" << strerror(ret) << "`)" << endl;
		return;
	}

	policy = SCHED_RR;
	int min_prio = sched_get_priority_min(policy), max_prio = sched_get_priority_max(policy);
	if (min_prio<0) min_prio=1; // Just in case of error to calls above (!)
	if (max_prio<0) max_prio=99;

	int prio = 0;
	switch(priority)
	{
		case tpLowests: prio=min_prio; break;
		case tpLower :  prio=(max_prio+3*min_prio)/4; break;
		case tpLow :    prio=(max_prio+2*min_prio)/3; break;
		case tpNormal:  prio=(max_prio+min_prio  )/2; break;
		case tpHigh :   prio=(2*max_prio+min_prio)/3; break;
		case tpHigher:  prio=(3*max_prio+min_prio)/4; break;
		case tpHighest: prio=max_prio; break;
	}

	param.sched_priority = prio;
	if (0!=(ret=pthread_setschedparam(tid, policy, &param))) {
		cerr << "[mrpt::system::changeThreadPriority] Warning: Failed call to pthread_setschedparam (error: `" << strerror(ret) << "`)" << endl;
		return;
	}

#endif
}

/*---------------------------------------------------------------
					changeCurrentProcessPriority
---------------------------------------------------------------*/
void BASE_IMPEXP mrpt::system::changeCurrentProcessPriority( TProcessPriority priority  )
{
#ifdef MRPT_OS_WINDOWS
	DWORD dwPri;
	switch (priority)
	{
	case ppIdle:	dwPri = IDLE_PRIORITY_CLASS; break;
	case ppNormal:	dwPri = NORMAL_PRIORITY_CLASS; break;
	case ppHigh:	dwPri = HIGH_PRIORITY_CLASS; break;
	case ppVeryHigh: dwPri= REALTIME_PRIORITY_CLASS; break;
	default:
		THROW_EXCEPTION("Invalid priority value");
	}
	SetPriorityClass( GetCurrentProcess(), dwPri );
#else
	int nice_val;
	switch (priority)
	{
	case ppIdle:     nice_val =+19; break;
	case ppNormal:   nice_val =  0; break;
	case ppHigh:     nice_val =-10; break;
	case ppVeryHigh: nice_val =-20; break;
	default:
		THROW_EXCEPTION("Invalid priority value");
	}
	errno=0;
	const int ret = nice(nice_val);
	if (ret==-1 && errno==EPERM) {
		std::cerr << "[mrpt::system::changeCurrentProcessPriority] Error calling nice(): Not enough permissions.\n";
	}
#endif
}

/*---------------------------------------------------------------
					mrpt::system::getCurrentThreadTimes
---------------------------------------------------------------*/
void mrpt::system::getCurrentThreadTimes(
	time_t			&creationTime,
	time_t			&exitTime,
	double			&cpuTime )
{
	MRPT_START

#ifdef MRPT_OS_WINDOWS
	FILETIME	timCreat,timExit, timKernel, timUser;
	uint64_t	t;

	HANDLE threadHandle;

#if !defined(HAVE_OPENTHREAD) // defined(_MSC_VER) && (_MSC_VER<1300)
	// In MSVC6/GCC the ID is just the HANDLE:
	threadHandle = reinterpret_cast<HANDLE>( mrpt::system::getCurrentThreadId() );
#else
	// Get the handle from the ID:
	threadHandle = OpenThread( READ_CONTROL | THREAD_QUERY_INFORMATION, FALSE,  GetCurrentThreadId() );  // threadId);
	if (!threadHandle)	 THROW_EXCEPTION("Cannot open the thread with the given 'threadId'");
#endif

	if (!GetThreadTimes( threadHandle , &timCreat, &timExit, &timKernel, &timUser ))
	{
		CloseHandle(threadHandle);
		THROW_EXCEPTION("Error accessing thread times!");
	}

#if defined(HAVE_OPENTHREAD) // _MSC_VER) && (_MSC_VER>=1300)
	// From OpenThread...
	CloseHandle(threadHandle);
#endif

	// Formula is derived from:
	//  http://support.microsoft.com/kb/167296
	t = (((uint64_t)timCreat.dwHighDateTime) << 32) | timCreat.dwLowDateTime;
	creationTime = (t - 116444736000000000ULL)/10000000;

	t = (((uint64_t)timExit.dwHighDateTime) << 32) | timExit.dwLowDateTime;
	exitTime = (t - 116444736000000000ULL)/10000000;

	// CPU time is user+kernel:
	int64_t	t1 = (((uint64_t)timKernel.dwHighDateTime) << 32) | timKernel.dwLowDateTime;
	int64_t	t2 = (((uint64_t)timUser.dwHighDateTime) << 32) | timUser.dwLowDateTime;

	cpuTime = ((double)(t1+t2)) * 100e-9;	// FILETIME counts intervals of 100ns

#endif

#ifdef MRPT_OS_LINUX
	MRPT_UNUSED_PARAM(creationTime);
	MRPT_UNUSED_PARAM(exitTime);
	// Unix:
#	ifdef HAVE_GETTID
		pid_t 	id = gettid();
#	else
		// gettid is:
		//  186 in 64bit
		//  224 in 32bit
		#if MRPT_WORD_SIZE==64
			pid_t 	id = (long int)syscall(186);
		#elif MRPT_WORD_SIZE==32
			pid_t 	id = (long int)syscall(224);
		#else
			#error MRPT_WORD_SIZE must be 32 or 64.
		#endif
#	endif

	// (JL) Refer to: /usr/src/linux/fs/proc/array.c
	long unsigned 	tms_utime=0, tms_stime=0;
	ifstream is(format("/proc/self/task/%i/stat", id).c_str() );

	if (is.is_open())
	{
		string s;
		getline(is,s);

		size_t idx = s.find(")");

		if (idx!=string::npos)
		{
			vector_string	tokens;
			mrpt::system::tokenize( string(s.c_str()+idx+1)," ",tokens);

			if (tokens.size()>=13)
			{
				sscanf(tokens[11].c_str(), "%lu" ,&tms_utime);
				sscanf(tokens[12].c_str(), "%lu", &tms_stime);
			}
		}
	}

	// Compute cpuTime:
	double clockTicksPerSecond = (double)sysconf(_SC_CLK_TCK);
	if (clockTicksPerSecond>0)
		cpuTime = (tms_utime + tms_stime) / clockTicksPerSecond;
#endif

#ifdef MRPT_OS_APPLE
	thread_basic_info info;
	mach_msg_type_number_t count = THREAD_BASIC_INFO_COUNT;
	if(thread_info(mach_thread_self(), THREAD_BASIC_INFO, (thread_info_t)&info, &count)==0)
	{
		double utime = info.user_time.seconds + info.user_time.microseconds * 1e-6;
		double stime = info.system_time.seconds + info.system_time.microseconds * 1e-6;
		cpuTime = utime + stime;
	}
#endif

	MRPT_END
}

/*---------------------------------------------------------------
					launchProcess
  ---------------------------------------------------------------*/
bool mrpt::system::launchProcess( const std::string & command )
{
#ifdef MRPT_OS_WINDOWS
	STARTUPINFOA			SI;
	PROCESS_INFORMATION		PI;

	memset(&SI,0,sizeof(STARTUPINFOA) );
	SI.cb = sizeof(STARTUPINFOA);

	if (CreateProcessA( NULL, (LPSTR)command.c_str(), NULL, NULL, true, 0,	NULL, NULL,	&SI, &PI) )
	{
		// Wait:
		WaitForSingleObject( PI.hProcess, INFINITE );
		return true;
	} // End of process executed OK
	else
	{
		char str[300];
		DWORD e = GetLastError();

		FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM,0,e,0,str,sizeof(str), NULL);

		// ERROR:
		std::cerr << "[launchProcess] Couldn't spawn process. Error msg: " << str << std::endl;
		return false;
	}
#else
	return 0== ::system(command.c_str());
#endif
} // end launchProcess

// Return number of processors in the system
unsigned int mrpt::system::getNumberOfProcessors()
{
    static unsigned int ret = 0;

    if (!ret)
    {
#ifdef MRPT_OS_WINDOWS
        SYSTEM_INFO si;  // Requires Win200 or above.
        GetSystemInfo(&si);
        ret=si.dwNumberOfProcessors;
        if (ret<1) ret=1;
#elif defined(MRPT_OS_APPLE)
        size_t len=sizeof(int);
        if(sysctlbyname("hw.logicalcpu", &ret, &len, NULL, 0) != 0)
            ret = 1; // failed
#else
        // This assumes a Linux kernel 2.6
        ifstream f;
        f.open("/proc/cpuinfo");
        if (!f.is_open())
            return 1; // No info...

        std::string lin;
        unsigned int nProc = 0;
        while (!f.fail() && !f.eof())
        {
            std::getline(f,lin);
            if (!f.fail() && !f.eof())
                if (lin.find("processor")!=std::string::npos)
                    nProc++;
        }
        ret = nProc ? nProc : 1;
#endif
    }
    return ret;
}

/*---------------------------------------------------------------
					exitThread
  ---------------------------------------------------------------*/
void mrpt::system::exitThread() MRPT_NO_THROWS
{
#ifdef MRPT_OS_WINDOWS
	ExitThread(0);
#else
	pthread_exit(NULL);
#endif
}

/*---------------------------------------------------------------
					terminateThread
  ---------------------------------------------------------------*/
void mrpt::system::terminateThread(TThreadHandle &threadHandle) MRPT_NO_THROWS
{
	if (threadHandle.isClear()) return; // done

#ifdef MRPT_OS_WINDOWS
	TerminateThread(threadHandle.hThread, DWORD(-1));
#elif defined(MRPT_OS_APPLE)
	pthread_cancel(reinterpret_cast<pthread_t>(threadHandle.idThread));
#else
	pthread_cancel(threadHandle.idThread);
#endif
	threadHandle.clear();
}
