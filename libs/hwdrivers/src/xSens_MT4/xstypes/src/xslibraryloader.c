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
	return GetProcAddress(thisp->m_handle, functionName);
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
