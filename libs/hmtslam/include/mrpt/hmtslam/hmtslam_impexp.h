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

#ifndef _HMTIMPEXP_H
#define _HMTIMPEXP_H

#if defined(MRPT_OS_WINDOWS)
    /*
       __declspec works in BC++ 5 and later, Watcom C++ 11.0 and later as well
       as VC++ and gcc
     */
#    if defined(_MSC_VER) || defined(__BORLANDC__) || defined(__GNUC__) || defined(__WATCOMC__)
#        define HMTEXPORT __declspec(dllexport)
#        define HMTIMPORT __declspec(dllimport)
#    else /* compiler doesn't support __declspec() */
#        define HMTEXPORT
#        define HMTIMPORT
#    endif
#elif defined(MRPT_OS_OS2)		/* was __WXPM__ */
#    if defined (__WATCOMC__)
#        define HMTEXPORT __declspec(dllexport)
        /*
           __declspec(dllimport) prepends __imp to imported symbols. We do NOT
           want that!
         */
#        define HMTIMPORT
#    elif defined(__EMX__)
#        define HMTEXPORT
#        define HMTIMPORT
#    elif (!(defined(__VISAGECPP__) && (__IBMCPP__ < 400 || __IBMC__ < 400 )))
#        define HMTEXPORT _Export
#        define HMTIMPORT _Export
#    endif
#elif defined(MRPT_OS_APPLE)
#    ifdef __MWERKS__
#        define HMTEXPORT __declspec(export)
#        define HMTIMPORT __declspec(import)
#    endif
#elif defined(__CYGWIN__)
#    define HMTEXPORT __declspec(dllexport)
#    define HMTIMPORT __declspec(dllimport)
#endif

/* for other platforms/compilers we don't anything */
#ifndef HMTEXPORT
#    define HMTEXPORT
#    define HMTIMPORT
#endif

/*
   HMTSLAM_IMPEXP maps to export declaration when building the DLL, to import
   declaration if using it or to nothing at all if we are not compiling as DLL
 */
#if defined(MRPT_BUILT_AS_DLL)
#	if defined(mrpt_hmtslam_EXPORTS)  /* Building the DLL */
#		define HMTSLAM_IMPEXP HMTEXPORT
#	else  /* Using the DLL */
#		define HMTSLAM_IMPEXP HMTIMPORT
#	endif
#else /* not making nor using DLL */
#    define HMTSLAM_IMPEXP
#endif

#endif /*  end of _UTILSIMPEXP_H */
