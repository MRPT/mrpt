/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#ifndef BAYES_link_pragmas_H
#define BAYES_link_pragmas_H

#include <mrpt/config.h>
#include <mrpt/utils/boost_join.h>

// ** Important! **
// In each mrpt library, search and replace:
//  MRPT_XXX_EXPORT, MRPT_XXX_IMPORT
//  BAYES_IMPEXP, mrpt_xxx_EXPORTS

// If we are building the DLL (_EXPORTS), do not link against the .lib files:
#if !defined(mrpt_bayes_EXPORTS) && (defined(_MSC_VER) || defined(__BORLANDC__))
#	if defined(_DEBUG)
#		pragma comment (lib, BOOST_JOIN( BOOST_JOIN("libmrpt-bayes",MRPT_VERSION_POSTFIX),"-dbg.lib"))
#	else
#		pragma comment (lib, BOOST_JOIN( BOOST_JOIN("libmrpt-bayes",MRPT_VERSION_POSTFIX),".lib"))
#	endif
#endif




/*   The macros below for DLL import/export are required for Windows only.
    Mostly all the definitions in this file are copied or at least topod
     on the file wx/dlimpexp.h, written by Vadim Zeitlin and published
	 under the wxWindows licence.
*/
#if defined(MRPT_OS_WINDOWS)
    /*
       __declspec works in BC++ 5 and later, Watcom C++ 11.0 and later as well
       as VC++ and gcc
     */
#    if defined(_MSC_VER) || defined(__BORLANDC__) || defined(__GNUC__) || defined(__WATCOMC__)
#        define MRPT_BAYES_EXPORT __declspec(dllexport)
#        define MRPT_BAYES_IMPORT __declspec(dllimport)
#    else /* compiler doesn't support __declspec() */
#        define MRPT_BAYES_EXPORT
#        define MRPT_BAYES_IMPORT
#    endif
#elif defined(MRPT_OS_OS2)		/* was __WXPM__ */
#    if defined (__WATCOMC__)
#        define MRPT_BAYES_EXPORT __declspec(dllexport)
        /*
           __declspec(dllimport) prepends __imp to imported symbols. We do NOT
           want that!
         */
#        define MRPT_BAYES_IMPORT
#    elif defined(__EMX__)
#        define MRPT_BAYES_EXPORT
#        define MRPT_BAYES_IMPORT
#    elif (!(defined(__VISAGECPP__) && (__IBMCPP__ < 400 || __IBMC__ < 400 )))
#        define MRPT_BAYES_EXPORT _Export
#        define MRPT_BAYES_IMPORT _Export
#    endif
#elif defined(MRPT_OS_APPLE)
#    ifdef __MWERKS__
#        define MRPT_BAYES_EXPORT __declspec(export)
#        define MRPT_BAYES_IMPORT __declspec(import)
#    endif
#elif defined(__CYGWIN__)
#    define MRPT_BAYES_EXPORT __declspec(dllexport)
#    define MRPT_BAYES_IMPORT __declspec(dllimport)
#endif

/* for other platforms/compilers we don't anything */
#ifndef MRPT_BAYES_EXPORT
#    define MRPT_BAYES_EXPORT
#    define MRPT_BAYES_IMPORT
#endif

/*  Macros that map to export declaration when building the DLL, to import
	declaration if using it or to nothing at all if we are not compiling as DLL */
#if defined(MRPT_BUILT_AS_DLL)
#	if defined(mrpt_bayes_EXPORTS)  /* Building the DLL */
#		define BAYES_IMPEXP MRPT_BAYES_EXPORT
#	else  /* Using the DLL */
#		define BAYES_IMPEXP MRPT_BAYES_IMPORT
#	endif
#else /* not making nor using DLL */
#    define BAYES_IMPEXP 
#endif


#endif
