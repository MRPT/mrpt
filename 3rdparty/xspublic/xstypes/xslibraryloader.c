
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

#include "xslibraryloader.h"
#include "xsstring.h"

#ifdef __GNUC__
#include <dlfcn.h>
#elif defined(_MSC_VER)
#include <Windows.h>
#endif

/*! \brief Dynamically load a library

	\param[in,out] thisp the XsLibraryLoader object handle
	\param[in] libraryName the name of the library to load. The library
		should be present in the current search path, or be specified by a path
	\return non-zero if the library could be loaded, zero otherwise
*/
int XsLibraryLoader_load(XsLibraryLoader* thisp, const XsString* libraryName)
{
#ifdef __GNUC__
	if (XsLibraryLoader_isLoaded(thisp))
		return 0;
	thisp->m_handle = dlopen(libraryName->m_data, RTLD_LAZY);
#elif defined(_MSC_VER)
	wchar_t *libraryNameW;
	XsSize required;
	if (XsLibraryLoader_isLoaded(thisp))
		return 0;
	required = XsString_copyToWCharArray(libraryName, NULL, 0);
	libraryNameW = (wchar_t*)malloc(required * sizeof(wchar_t));

	(void)XsString_copyToWCharArray(libraryName, libraryNameW, required);

	thisp->m_handle = LoadLibrary(libraryNameW);

	free(libraryNameW);
#endif
	return XsLibraryLoader_isLoaded(thisp);
}

/*! \brief Resolve a function from the library

	\param[in] thisp the library handle
	\param[in] functionName the name of the function
	\return a pointer to the resolved function, may be NULL if no function could be resolved
*/
void* XsLibraryLoader_resolve(const XsLibraryLoader* thisp, const char* functionName)
{
#ifdef __GNUC__
	return dlsym(thisp->m_handle, functionName);
#elif defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable: 4152)
	return GetProcAddress(thisp->m_handle, functionName);
#pragma warning(pop)
#endif
}

/*! \brief Unload the loaded library

	\param[in,out] thisp the library handle
	\return zero on failure, non-zero otherwise
*/
int XsLibraryLoader_unload(XsLibraryLoader* thisp)
{
	void *handle = thisp->m_handle;
	thisp->m_handle = NULL;
	if (handle)
	{
#ifdef __GNUC__
		return dlclose(handle) == 0;
#elif defined(_MSC_VER)
		return FreeLibrary(handle) != 0;
#endif
	}
	return 0;
}

/*! \brief Check if a library is loaded

	\param[in] thisp the library handle
	\return zero if nothing is loaded, non-zero otherwise
*/
int XsLibraryLoader_isLoaded(const XsLibraryLoader* thisp)
{
	return thisp->m_handle != NULL;
}

/*! \brief Get an error string after a failure occurred
	\param[in,out] error the string to fill with the result error
*/
void XsLibraryLoader_getErrorString(XsString* error)
{
#ifdef __GNUC__
	XsString_assignCharArray(error, dlerror());
#elif defined(_MSC_VER)
	LPTSTR errorText = NULL;
	(void)FormatMessageW(FORMAT_MESSAGE_FROM_SYSTEM
			|FORMAT_MESSAGE_ALLOCATE_BUFFER
			|FORMAT_MESSAGE_IGNORE_INSERTS,
			NULL,
			GetLastError(),
			MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
			(LPTSTR)&errorText,
			0,
			NULL);

	XsString_assignWCharArray(error, errorText);
	LocalFree(errorText);
#endif
}
