/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSLIBRARYLOADER_H
#define XSLIBRARYLOADER_H

#include "xstypesconfig.h"
#include "xsstring.h"

struct XsLibraryLoader;

#ifdef __cplusplus
extern "C" {
#else
#define XSLIBRARYLOADER_INITIALIZER { NULL }
typedef struct XsLibraryLoader XsLibraryLoader;
#endif

XSTYPES_DLL_API int XsLibraryLoader_load(XsLibraryLoader* thisp, const XsString* libraryName);
XSTYPES_DLL_API void* XsLibraryLoader_resolve(const XsLibraryLoader* thisp, const char *functionName);
XSTYPES_DLL_API int XsLibraryLoader_unload(XsLibraryLoader* thisp);
XSTYPES_DLL_API int XsLibraryLoader_isLoaded(const XsLibraryLoader* thisp);
XSTYPES_DLL_API void XsLibraryLoader_getErrorString(XsString* error);

#ifdef __cplusplus
}
#endif

/*! \brief The Xsens dynamic library loader base class
*/
struct XsLibraryLoader {
#ifdef __cplusplus
public:
	/*! \brief Create a library loader */
	inline XsLibraryLoader() :
		m_handle(NULL)
	{
		// avoid compiler warnings about
		// an unused handle. It is used in the c implementations
		(void)m_handle;
	}

	/*! \brief Destroy a library loader */
	inline ~XsLibraryLoader()
	{
		unload();
	}

	/*! \brief Load the library
	  \param[in] libraryName the name of the library to load
	  \return true if the library could be loaded, false otherwise
	*/
	inline bool load(const XsString& libraryName)
	{
		return XsLibraryLoader_load(this, &libraryName) != 0;
	}

	/*! \brief Return true if a library has been loaded

	  \return true if a library has been loaded, false otherwise
	*/
	inline bool isLoaded() const
	{
		return XsLibraryLoader_isLoaded(this) != 0;
	}

	/*! \brief Resolve a function from the library

	  \param[in] functionName the name of the function to resolve
	  \return a pointer to the resolved function, NULL if nothing could be resolved
	*/
	inline void* resolve(const char *functionName) const
	{
		return XsLibraryLoader_resolve(this, functionName);
	}

	/*! \brief Unload the loaded library
	*/
	inline void unload() throw()
	{
		XsLibraryLoader_unload(this);
	}

	/*! \brief Return a string describing the error that occurred

	  Use this function after a function returned with an error to
	  receive some extra information about what went wrong.

	  \returns a string describing the error that occurred
	*/
	inline static XsString errorString()
	{
		XsString rv;
		XsLibraryLoader_getErrorString(&rv);
		return rv;
	}
private:
#endif
	void* m_handle;
};

#endif //XSLIBRARYLOADER_H
