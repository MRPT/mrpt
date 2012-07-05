/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

/* This file should be included from utils_defs.h only!
*/
#ifndef _IAMINUTILSDEFS_H
#error Do not include this file manually
#endif

/*   This file defines macros for DLL import/export, required for
       Windows only.

    Mostly all the definitions in this file are copied or at least based
     on the file wx/dlimpexp.h, written by Vadim Zeitlin and published
	 under the wxWindows licence.
*/

#ifndef _HWIMPEXP_H
#define _HWIMPEXP_H


#if defined(MRPT_OS_WINDOWS)
    /*
       __declspec works in BC++ 5 and later, Watcom C++ 11.0 and later as well
       as VC++ and gcc
     */
#    if defined(_MSC_VER) || defined(__BORLANDC__) || defined(__GNUC__) || defined(__WATCOMC__)
#        define HWEXPORT __declspec(dllexport)
#        define HWIMPORT __declspec(dllimport)
#    else /* compiler doesn't support __declspec() */
#        define HWEXPORT
#        define HWIMPORT
#    endif
#elif defined(MRPT_OS_OS2)		/* was __WXPM__ */
#    if defined (__WATCOMC__)
#        define HWEXPORT __declspec(dllexport)
        /*
           __declspec(dllimport) prepends __imp to imported symbols. We do NOT
           want that!
         */
#        define HWIMPORT
#    elif defined(__EMX__)
#        define HWEXPORT
#        define HWIMPORT
#    elif (!(defined(__VISAGECPP__) && (__IBMCPP__ < 400 || __IBMC__ < 400 )))
#        define HWEXPORT _Export
#        define HWIMPORT _Export
#    endif
#elif defined(MRPT_OS_APPLE)
#    ifdef __MWERKS__
#        define HWEXPORT __declspec(export)
#        define HWIMPORT __declspec(import)
#    endif
#elif defined(__CYGWIN__)
#    define HWEXPORT __declspec(dllexport)
#    define HWIMPORT __declspec(dllimport)
#endif

/* for other platforms/compilers we don't anything */
#ifndef HWEXPORT
#    define HWEXPORT
#    define HWIMPORT
#endif

/*
   HWDRIVERS_IMPEXP maps to export declaration when building the DLL, to import
   declaration if using it or to nothing at all if we are not compiling as DLL
 */
#if defined(MRPT_BUILT_AS_DLL)
#	if defined(mrpt_hwdrivers_EXPORTS)  /* Building the DLL */
#		define HWDRIVERS_IMPEXP HWEXPORT
#	else  /* Using the DLL */
#		define HWDRIVERS_IMPEXP HWIMPORT
#	endif
#else /* not making nor using DLL */
#    define HWDRIVERS_IMPEXP
#endif

#endif /*  end of _UTILSIMPEXP_H */
