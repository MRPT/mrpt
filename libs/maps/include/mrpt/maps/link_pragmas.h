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

#ifndef maps_link_pragmas_H
#define maps_link_pragmas_H

#include <mrpt/config.h>
#include <mrpt/utils/boost_join.h>

// ** Important! **
// In each mrpt library, search and replace:
//  MRPT_XXX_EXPORT, MRPT_XXX_IMPORT
//  MAPS_IMPEXP, mrpt_xxx_EXPORTS

// If we are building the DLL (_EXPORTS), do not link against the .lib files:
#if !defined(mrpt_maps_EXPORTS) && (defined(_MSC_VER) || defined(__BORLANDC__))
#	if defined(_DEBUG)
#		pragma comment (lib, BOOST_JOIN( BOOST_JOIN("libmrpt-maps",MRPT_VERSION_POSTFIX),"-dbg.lib"))
#	else
#		pragma comment (lib, BOOST_JOIN( BOOST_JOIN("libmrpt-maps",MRPT_VERSION_POSTFIX),".lib"))
#	endif
#endif




/*   The macros below for DLL import/export are required for Windows only.
    Mostly all the definitions in this file are copied or at least mapsd
     on the file wx/dlimpexp.h, written by Vadim Zeitlin and published
	 under the wxWindows licence.
*/
#if defined(MRPT_OS_WINDOWS)
    /*
       __declspec works in BC++ 5 and later, Watcom C++ 11.0 and later as well
       as VC++ and gcc
     */
#    if defined(_MSC_VER) || defined(__BORLANDC__) || defined(__GNUC__) || defined(__WATCOMC__)
#        define MRPT_MAPS_EXPORT __declspec(dllexport)
#        define MRPT_MAPS_IMPORT __declspec(dllimport)
#    else /* compiler doesn't support __declspec() */
#        define MRPT_MAPS_EXPORT
#        define MRPT_MAPS_IMPORT
#    endif
#elif defined(MRPT_OS_OS2)		/* was __WXPM__ */
#    if defined (__WATCOMC__)
#        define MRPT_MAPS_EXPORT __declspec(dllexport)
        /*
           __declspec(dllimport) prepends __imp to imported symbols. We do NOT
           want that!
         */
#        define MRPT_MAPS_IMPORT
#    elif defined(__EMX__)
#        define MRPT_MAPS_EXPORT
#        define MRPT_MAPS_IMPORT
#    elif (!(defined(__VISAGECPP__) && (__IBMCPP__ < 400 || __IBMC__ < 400 )))
#        define MRPT_MAPS_EXPORT _Export
#        define MRPT_MAPS_IMPORT _Export
#    endif
#elif defined(MRPT_OS_APPLE)
#    ifdef __MWERKS__
#        define MRPT_MAPS_EXPORT __declspec(export)
#        define MRPT_MAPS_IMPORT __declspec(import)
#    endif
#elif defined(__CYGWIN__)
#    define MRPT_MAPS_EXPORT __declspec(dllexport)
#    define MRPT_MAPS_IMPORT __declspec(dllimport)
#endif

/* for other platforms/compilers we don't anything */
#ifndef MRPT_MAPS_EXPORT
#    define MRPT_MAPS_EXPORT
#    define MRPT_MAPS_IMPORT
#endif

/*  Macros that map to export declaration when building the DLL, to import
	declaration if using it or to nothing at all if we are not compiling as DLL */
#if defined(MRPT_BUILT_AS_DLL)
#	if defined(mrpt_maps_EXPORTS)  /* Building the DLL */
#		define MAPS_IMPEXP MRPT_MAPS_EXPORT
#	else  /* Using the DLL */
#		define MAPS_IMPEXP MRPT_MAPS_IMPORT
#	endif
#else /* not making nor using DLL */
#    define MAPS_IMPEXP 
#endif


#endif
