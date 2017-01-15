/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/utils/core_defs.h>
#include <mrpt/system/memory.h>
#include <cstdio>
#include <cstring>
#include <cstdlib> // realloc(), posix_memalign()...

#ifdef MRPT_OS_APPLE
#include <mach/mach_init.h>
#include <mach/task.h>
#endif

using namespace mrpt;
using namespace mrpt::system;
using namespace std;

// Management of aligned memory for efficiency:
// If we have "posix_memalign", use it, then realloc / free as usual. (GCC/Linux)
// If we have "_aligned_malloc", use it, then _aligned_realloc/_aligned_free (MSVC, MinGW)

/** Returns an aligned memory block.
  * \param alignment The desired alignment, typ. 8 or 16 bytes. 1 means no alignment required. It must be a power of two.
  * \sa aligned_free, aligned_realloc
  * \note Based on code by William Chan
*/
void* mrpt::system::os::aligned_malloc(size_t bytes, size_t alignment)
{
#if defined(HAVE_ALIGNED_MALLOC)
#   if defined(__GNUC__) && !defined(__MINGW32__)
		return ::aligned_malloc(bytes,alignment);
#   else
        return _aligned_malloc(bytes,alignment);
#   endif
#elif defined(HAVE_POSIX_MEMALIGN)
	void *ptr=NULL;
	int ret = posix_memalign(&ptr,alignment,bytes);
	if (ret) THROW_EXCEPTION("posix_memalign returned an error.");
	return ptr;
#else
	// We don't have aligned memory:
	return ::malloc(bytes);
#endif
}

/** Frees a memory block reserved by aligned_malloc.
  * \param alignment The desired alignment, typ. 8 or 16 bytes. 1 means no alignment required.
  * If old_ptr is NULL, a new block will be reserved from scratch.
  * \sa aligned_malloc, aligned_free
  */
void* mrpt::system::os::aligned_realloc(void* old_ptr, size_t bytes, size_t alignment)
{
#if defined(HAVE_ALIGNED_MALLOC)
#   if defined(__GNUC__) && !defined(__MINGW32__)
	return ::aligned_realloc(old_ptr,bytes,alignment);
#   else
	return _aligned_realloc(old_ptr,bytes,alignment);
#   endif
#elif defined(HAVE_POSIX_MEMALIGN)
	MRPT_UNUSED_PARAM(alignment);
	return ::realloc(old_ptr,bytes);
#else
	MRPT_UNUSED_PARAM(alignment);
	// We don't have aligned memory:
	return ::realloc(old_ptr,bytes);
#endif
}

/** Frees a memory block reserved by aligned_malloc
  * \sa aligned_malloc
  */
void mrpt::system::os::aligned_free(void* p)
{
#if defined(HAVE_ALIGNED_MALLOC)
#   if defined(__GNUC__) && !defined(__MINGW32__)
	::aligned_free(p);
#   else
	_aligned_free(p);
#   endif
#elif defined(HAVE_POSIX_MEMALIGN)
	free(p);
#else
	// We don't have aligned memory:
	free(p);
#endif
}


#ifdef  MRPT_OS_WINDOWS
#include <windows.h>
	// Windows:
	typedef struct _PROCESS_MEMORY_COUNTERS {
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
	} PROCESS_MEMORY_COUNTERS,
	*PPROCESS_MEMORY_COUNTERS;

	namespace mrpt
	{
		namespace system
		{
			/** This is an auxiliary class for mrpt::system::getMemoryUsage() under Windows.
			  *  It loads in runtime PSAPI.DLL. This is to avoid problems in some platforms, i.e Windows 2000,
			  *  where this DLL must not be present.
			  */
			class CAuxPSAPI_Loader
			{
			protected:
				typedef BOOL (WINAPI *TGetProcessMemoryInfo)(
				  HANDLE Process,
				  PPROCESS_MEMORY_COUNTERS ppsmemCounters,
				  DWORD cb );

				TGetProcessMemoryInfo		m_ptr;

			public:
				HMODULE m_dll;

				CAuxPSAPI_Loader()
				{
					m_ptr = NULL;

					m_dll = LoadLibraryA("PSAPI.DLL");
					if (m_dll)
					{
						m_ptr = (TGetProcessMemoryInfo) GetProcAddress(m_dll,"GetProcessMemoryInfo");
					}
				}
				~CAuxPSAPI_Loader()
				{
					if (m_dll)
					{
						FreeLibrary(m_dll);
						m_dll = NULL;
						m_ptr = NULL;
					}
				}

				BOOL WINAPI GetProcessMemoryInfo(
				  HANDLE Process,
				  PPROCESS_MEMORY_COUNTERS ppsmemCounters,
				  DWORD cb )
				{
					try
					{
						if (!m_ptr)
								return false;
						else	return (*m_ptr )(Process,ppsmemCounters,cb);
					}
					catch(...)
					{
						return false;
					}
				}
			};
		}
	}

#endif

/*---------------------------------------------------------------
						getMemoryUsage
 ---------------------------------------------------------------*/
unsigned long  mrpt::system::getMemoryUsage()
{
	MRPT_START
	unsigned long MEM = 0;

#ifdef  MRPT_OS_WINDOWS
	// Windows:
	static CAuxPSAPI_Loader		PSAPI_LOADER;

	PROCESS_MEMORY_COUNTERS		pmc;
	pmc.cb = sizeof(pmc);

	if ( PSAPI_LOADER.GetProcessMemoryInfo( GetCurrentProcess(),&pmc,sizeof(pmc)  ) )
	{
		MEM = (long)pmc.PagefileUsage;
	}
#endif

#ifdef MRPT_OS_LINUX
	// Linux:
	//int page_size = getpagesize();

	FILE *f = ::fopen ("/proc/self/stat", "r");
	if (!f) return 0;

	// Note: some of these scanf specifiers would normally be 'long' versions if
	// not for the fact that we are using suppression (gcc warns).  see 'man
	// proc' for scanf specifiers and meanings.
	if (!::fscanf(f,
		"%*d %*s %*c %*d %*d %*d %*d %*d %*u %*u %*u %*u %*u %*u %*u %*d %*d %*d %*d %*d %*d %*u %lu", &MEM))
	{
		// Error parsing:
		MEM=0;
	}
	::fclose (f);
	//::system("cat /proc/self/statm");
#endif

#ifdef MRPT_OS_APPLE
	mach_task_basic_info info;
	mach_msg_type_number_t count = MACH_TASK_BASIC_INFO_COUNT;
	if(task_info(mach_task_self(), MACH_TASK_BASIC_INFO, (task_info_t)&info, &count)==0)
	{
		MEM=info.virtual_size;
	}
#endif
	return MEM;
	MRPT_END
}
