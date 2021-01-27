/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "system-precomp.h"	 // Precompiled headers
//
#include <mrpt/config.h>  // for MRPT_OS_LINUX
#include <mrpt/core/exceptions.h>  // for MRPT_END, MRPT_START, MRPT_UNUSE...
#include <mrpt/system/memory.h>

#include <cstdio>  // for size_t, fclose, fopen, fscanf, FILE
#include <cstdlib>	// for free, realloc
#include <exception>  // for exception

#ifdef __APPLE__
#include <mach/mach_init.h>
#include <mach/task.h>
#endif

#ifdef MRPT_OS_LINUX
#include <unistd.h>	 // sysconf()
#endif

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
// Windows:
typedef struct _PROCESS_MEMORY_COUNTERS
{
	DWORD cb;
	DWORD PageFaultCount;
	SIZE_T PeakWorkingSetSize;
	SIZE_T WorkingSetSize;
	SIZE_T QuotaPeakPagedPoolUsage;
	SIZE_T QuotaPagedPoolUsage;
	SIZE_T QuotaPeakNonPagedPoolUsage;
	SIZE_T QuotaNonPagedPoolUsage;
	SIZE_T PagefileUsage;
	SIZE_T PeakPagefileUsage;
} PROCESS_MEMORY_COUNTERS, *PPROCESS_MEMORY_COUNTERS;

using namespace mrpt;
using namespace mrpt::system;
using namespace std;

namespace mrpt
{
namespace system
{
/** This is an auxiliary class for mrpt::system::getMemoryUsage() under Windows.
 *  It loads in runtime PSAPI.DLL. This is to avoid problems in some platforms,
 * i.e Windows 2000,
 *  where this DLL must not be present.
 */
class CAuxPSAPI_Loader
{
   protected:
	typedef BOOL(WINAPI* TGetProcessMemoryInfo)(
		HANDLE Process, PPROCESS_MEMORY_COUNTERS ppsmemCounters, DWORD cb);

	TGetProcessMemoryInfo m_ptr;

   public:
	HMODULE m_dll;

	CAuxPSAPI_Loader()
	{
		m_ptr = nullptr;

		m_dll = LoadLibraryA("PSAPI.DLL");
		if (m_dll)
		{
			m_ptr = (TGetProcessMemoryInfo)GetProcAddress(
				m_dll, "GetProcessMemoryInfo");
		}
	}
	~CAuxPSAPI_Loader()
	{
		if (m_dll)
		{
			FreeLibrary(m_dll);
			m_dll = nullptr;
			m_ptr = nullptr;
		}
	}

	BOOL WINAPI GetProcessMemoryInfo(
		HANDLE Process, PPROCESS_MEMORY_COUNTERS ppsmemCounters, DWORD cb)
	{
		try
		{
			if (!m_ptr) return false;
			else
				return (*m_ptr)(Process, ppsmemCounters, cb);
		}
		catch (...)
		{
			return false;
		}
	}
};
}  // namespace system
}  // namespace mrpt

#endif

/*---------------------------------------------------------------
						getMemoryUsage
 ---------------------------------------------------------------*/
unsigned long mrpt::system::getMemoryUsage()
{
	MRPT_START
	unsigned long MEM = 0;

#ifdef _WIN32
	// Windows:
	static CAuxPSAPI_Loader PSAPI_LOADER;

	PROCESS_MEMORY_COUNTERS pmc;
	pmc.cb = sizeof(pmc);

	if (PSAPI_LOADER.GetProcessMemoryInfo(
			GetCurrentProcess(), &pmc, sizeof(pmc)))
	{ MEM = (long)pmc.PagefileUsage; }
#endif

#ifdef MRPT_OS_LINUX
	FILE* f = ::fopen("/proc/self/statm", "r");
	if (!f) return 0;

	unsigned long mem_pages = 0;
	// see 'man proc' for docs on this
	if (!::fscanf(f, "%*d %*d %*d %*d %*d %lu %*d", &mem_pages))
	{
		// Error parsing.
		MEM = 0;
	}
	::fclose(f);

	// Multiply by the page size:
	static const long pagesize = ::sysconf(_SC_PAGE_SIZE);
	MEM = static_cast<decltype(MEM)>(pagesize) * mem_pages;

#endif

#ifdef __APPLE__
	mach_task_basic_info info;
	mach_msg_type_number_t count = MACH_TASK_BASIC_INFO_COUNT;
	if (task_info(
			mach_task_self(), MACH_TASK_BASIC_INFO, (task_info_t)&info,
			&count) == 0)
	{ MEM = info.virtual_size; }
#endif
	return MEM;
	MRPT_END
}
