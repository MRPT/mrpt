/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "xsthread.h"

#ifdef _MSC_VER
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
/*! \brief Set the name of the current thread to \a threadName
	\note Not implemented in POSIX
*/
void XSTYPES_DLL_API xsNameThisThread(const char* threadName)
{
	// no implementation for this in POSIX -- pthread_key_t should be known.
	// adding this function does remove some
	// checking from xs4 though.
	(void)threadName;
}
#endif

/*! @} */
