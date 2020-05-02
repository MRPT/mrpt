
//  Copyright (c) 2003-2019 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#include "xsthread.h"

#ifdef _WIN32
#define MS_VC_EXCEPTION 0x406D1388

#pragma pack(push,8)
// Copied from windows API
struct THREADNAME_INFO
{
	DWORD dwType;		// Must be 0x1000.
	LPCSTR szName;		// Pointer to name (in user addr space).
	DWORD dwThreadID;	// XsThread ID (-1=caller thread).
	DWORD dwFlags;		// Reserved for future use, must be zero.
};
#pragma pack(pop)
typedef struct THREADNAME_INFO THREADNAME_INFO;

/*! \addtogroup cinterface C Interface
	@{
*/

/*! \brief Set the name of the current thread to \a threadName */
void XSTYPES_DLL_API xsNameThisThread(const char* threadName)
{
	DWORD dwThreadID = GetCurrentThreadId();

	//Sleep(10);
	THREADNAME_INFO info;
	info.dwType = 0x1000;
	info.szName = threadName;
	info.dwThreadID = dwThreadID;
	info.dwFlags = 0;

	__try
	{
		RaiseException( MS_VC_EXCEPTION, 0, sizeof(info)/sizeof(ULONG_PTR), (ULONG_PTR*)&info );
	}
	__except(EXCEPTION_EXECUTE_HANDLER)
	{
	}
}
#else
#include <string.h>
#ifdef __APPLE__
inline static int pthread_setname_np2 (pthread_t __target_thread, const char *__name)
{
    (void) __target_thread;
    return pthread_setname_np(__name);
}
#else
/* Set thread name visible in the kernel and its interfaces.  */
extern int pthread_setname_np (pthread_t __target_thread, const char *__name);

inline static int pthread_setname_np2 (pthread_t __target_thread, const char *__name)
{
    return pthread_setname_np(__target_thread, __name);
}

#endif

/*! \brief Set the name of the current thread to \a threadName
	\note Not implemented in POSIX
*/
void XSTYPES_DLL_API xsNameThisThread(const char* threadName)
{
	if (pthread_setname_np2(xsGetCurrentThreadId(), threadName) == ERANGE)
	{
		char dup[16];
		strncpy(dup, threadName, 11);
		strncpy(dup+11, threadName + strlen(threadName)-4, 4);
		dup[15] = 0;
		pthread_setname_np2(xsGetCurrentThreadId(), dup);
	}
}

pthread_t XSTYPES_DLL_API xsStartThread(void *(func)(void *), void *param, void *pid) {
	(void)pid;
	pthread_t thread;
	if (pthread_create(&thread, NULL, func, param)) {
		return XSENS_INVALID_THREAD;
	}
	return thread;
}
#endif

/*! @} */
