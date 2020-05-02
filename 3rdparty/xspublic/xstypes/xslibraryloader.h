
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

#ifndef XSLIBRARYLOADER_H
#define XSLIBRARYLOADER_H

#include "xstypesconfig.h"
#include "xsstring.h"

struct XsLibraryLoader;

#ifdef __cplusplus
extern "C" {
#endif
#ifndef __cplusplus
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
	inline void unload() noexcept
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

#endif
