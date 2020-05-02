
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

#ifdef _WIN32
#	ifndef WINVER				// Allow use of features specific to Windows XP or later.
#		define WINVER 0x0501		// Change this to the appropriate value to target other versions of Windows.
#	endif

#	ifndef _WIN32_WINNT		// Allow use of features specific to Windows XP or later.
#		define _WIN32_WINNT 0x0501	// Change this to the appropriate value to target other versions of Windows.
#	endif

#	ifndef _WIN32_WINDOWS		// Allow use of features specific to Windows 98 or later.
#		define _WIN32_WINDOWS 0x0501 // Change this to the appropriate value to target Windows Me or later.
#	endif

#	ifndef _WIN32_IE			// Allow use of features specific to IE 6.0 or later.
#		define _WIN32_IE 0x0600	// Change this to the appropriate value to target other versions of IE.
#	endif
#	ifndef WIN32_LEAN_AND_MEAN
#		define WIN32_LEAN_AND_MEAN		// Exclude rarely-used stuff from Windows headers
#	endif
#	include <windows.h>
#else
#endif
#include "xstypesdynlib.h"
#include "xstime.h"
#include "xsstring.h"

#ifdef _MANAGED
#pragma managed(push, off)
#endif

// The so_* functions are also used by programs that use xstypes as a static library
// When the function contents change, check if this functionality is also required in these programs
void
#ifdef __GNUC__
__attribute__((constructor(65535)))
#endif
so_init(void)
{
	XsTime_initializeTime();
}

void
#ifdef __GNUC__
__attribute__((destructor))
#endif
so_fini(void)
{
}

#ifdef _WIN32
HMODULE g_hModule = 0;
BOOL APIENTRY DllMain( HMODULE hModule,
					   DWORD  ul_reason_for_call,
					   LPVOID lpvReserved
					 )
{
	g_hModule = hModule;

	switch (ul_reason_for_call)
	{
	case DLL_PROCESS_ATTACH:
		so_init();
		break;
	case DLL_THREAD_ATTACH:
		break;
	case DLL_THREAD_DETACH:
		break;
	case DLL_PROCESS_DETACH:
		// The lpvReserved parameter to DllMain is NULL if the DLL is being unloaded because of a call to FreeLibrary, it's non NULL if the DLL is being unloaded due to process termination.
		// in case of process termination, all the threads have already been destroyed and we'll just assume that windows will clean us up
		if (lpvReserved == NULL)
			so_fini();
		break;
	default:
		break;
	}
	return TRUE;
}
#endif

/*! \brief Return the path that the xsensdeviceapi.dll file is currently running from
	\param path Storage for the path
	\note This is a windows-only function
*/
XSTYPES_DLL_API void xstypesPath(XsString* path)
{
#ifdef XSENS_WINDOWS
	wchar_t filename[1024] = L"";
	if (!path)
		return;

	if (!GetModuleFileNameW(g_hModule, filename, 1024))
		XsString_destruct(path);
	else
		XsString_assignWCharArray(path, filename);
#else
	if (!path)
		return;
	XsString_destruct(path);
#endif
}
#ifdef _MANAGED
#pragma managed(pop)
#endif
