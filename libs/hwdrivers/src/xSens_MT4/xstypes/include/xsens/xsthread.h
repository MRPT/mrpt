/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSTHREAD_H
#define XSTHREAD_H

#include "xstime.h"
#if defined(XSENS_DEBUG) && defined(_MSC_VER)
#pragma warning (disable: 4985)
#include <intrin.h>
#endif

#ifndef __GNUC__
#pragma warning(push)
#pragma warning(disable: 4127)
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifdef XSENS_WINDOWS
	#ifndef WINVER				// Allow use of features specific to Windows XP or later.
	#define WINVER 0x0502		// Change this to the appropriate value to target other versions of Windows.
	#endif

	#ifndef _WIN32_WINNT		// Allow use of features specific to Windows XP or later.
	#define _WIN32_WINNT 0x0502	// Change this to the appropriate value to target other versions of Windows.
	#endif

	#ifndef _WIN32_WINDOWS		// Allow use of features specific to Windows 98 or later.
	#define _WIN32_WINDOWS 0x0410 // Change this to the appropriate value to target Windows Me or later.
	#endif

	#ifndef _WIN32_IE			// Allow use of features specific to IE 6.0 or later.
	#define _WIN32_IE 0x0600	// Change this to the appropriate value to target other versions of IE.
	#endif

	#include <windows.h>

	/*!	\addtogroup enums Global enumerations
		@{
	*/
	/*!	\brief Thread priorities for xsSetThreadPriority() and xsGetThreadPriority()
	*/
	enum XsThreadPriority {
		XS_THREAD_PRIORITY_LOWEST	= THREAD_PRIORITY_IDLE,
		XS_THREAD_PRIORITY_LOWER	= THREAD_PRIORITY_LOWEST,
		XS_THREAD_PRIORITY_LOW		= THREAD_PRIORITY_BELOW_NORMAL,
		XS_THREAD_PRIORITY_NORMAL	= THREAD_PRIORITY_NORMAL,
		XS_THREAD_PRIORITY_HIGH		= THREAD_PRIORITY_ABOVE_NORMAL,
		XS_THREAD_PRIORITY_HIGHER	= THREAD_PRIORITY_HIGHEST,
		XS_THREAD_PRIORITY_HIGHEST	= THREAD_PRIORITY_TIME_CRITICAL
	};
	/*! @} */

	// The components of the type of a thread function
	#define XSENS_THREAD_RETURN		DWORD
	#define XSENS_THREAD_TYPE		WINAPI
	#define XSENS_THREAD_PARAM		LPVOID

	#define XSENS_INVALID_THREAD	INVALID_HANDLE_VALUE

	/*! \brief Release the remainder of the timeslice so other operations can run.
		On Windows this is done using Sleep(0), since this is the most reliable method.
		SwitchToThread can cause delays since it does not allow the thread to resume on a different core.
		Sleep http://msdn.microsoft.com/en-us/library/ms686298%28v=vs.85%29.aspx
		SwitchToThread http://msdn.microsoft.com/en-us/library/ms686352%28v=vs.85%29.aspx
	*/
	#define xsYield()		Sleep(0)

	//! A handle for a thread
	typedef HANDLE XsThread;
#ifdef __cplusplus
	typedef ::DWORD XsThreadId;
#else
	typedef DWORD XsThreadId;
#endif

	//! Start a function as a thread
	#define xsStartThread(func,param,pid)	CreateThread(NULL,0,(LPTHREAD_START_ROUTINE) func,param,0,pid)

	XSTYPES_DLL_API void xsNameThisThread(const char* threadName);

	#define xsGetCurrentThreadId()			GetCurrentThreadId()
	#define xsGetCurrentThreadHandle()		GetCurrentThread()
	#define xsSuspendThread(thrd)			SuspendThread(thrd)
	#define xsResumeThread(thrd)			ResumeThread(thrd)
	#define xsSetThreadPriority(thrd,prio)	SetThreadPriority(thrd,prio)
	#define xsGetThreadPriority(thrd)		GetThreadPriority(thrd)

#else
#include <pthread.h>
#include <semaphore.h>
#include <errno.h>
//#define XSENS_USE_POSIX_LOCKING 1

	/*!	\addtogroup enums Global enumerations
		@{
	*/
	/*!	\brief Thread priorities for xsSetThreadPriority() and xsGetThreadPriority()
	*/
	enum XsThreadPriority {
		XS_THREAD_PRIORITY_LOWEST	= 0, //THREAD_PRIORITY_IDLE,
		XS_THREAD_PRIORITY_LOWER	= 1, //THREAD_PRIORITY_LOWEST,
		XS_THREAD_PRIORITY_LOW		= 2, //THREAD_PRIORITY_BELOW_NORMAL,
		XS_THREAD_PRIORITY_NORMAL	= 3, //THREAD_PRIORITY_NORMAL,
		XS_THREAD_PRIORITY_HIGH		= 4, //THREAD_PRIORITY_ABOVE_NORMAL,
		XS_THREAD_PRIORITY_HIGHER	= 5, //THREAD_PRIORITY_HIGHEST,
		XS_THREAD_PRIORITY_HIGHEST	= 6  //THREAD_PRIORITY_TIME_CRITICAL
	};
	/*! @} */

	// The components of the type of a thread function
	#define XSENS_THREAD_RETURN		void*	// DWORD
	#define XSENS_THREAD_TYPE		// WINAPI
	#define XSENS_THREAD_PARAM		void* // LPVOID

	#define XSENS_INVALID_THREAD	0	// INVALID_HANDLE_VALUE

	//! Release the remainder of the timeslice so other operations can run.
#ifdef __APPLE__
	#define xsYield()		pthread_yield_np()
#else
	#define xsYield()		pthread_yield()	// Sleep(0)
#endif

	void xsNameThisThread(const char* threadName);
	//! A handle for a thread
	typedef pthread_t XsThread;
	typedef pthread_t XsThreadId;

	//! Start a function as a thread
	pthread_t xsStartThread(void *(func)(void *), void *param, void *pid);
	#define xsGetCurrentThreadId()		pthread_self()
	#define xsSuspendThread(thrd)
	#define xsResumeThread(thrd)
	#define xsSetThreadPriority(thrd,prio)

#endif // _WIN32

#ifndef __GNUC__
#pragma warning(pop)
#endif

#ifdef __cplusplus
} // extern "C"
#endif

typedef enum XsThreadPriority XsThreadPriority;

#endif	// file guard
