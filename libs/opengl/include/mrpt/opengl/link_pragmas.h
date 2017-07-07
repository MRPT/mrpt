/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/config.h>

// ** Important! **
// In each mrpt library, search and replace:
//  MRPT_XXX_EXPORT, MRPT_XXX_IMPORT
//  OPENGL_IMPEXP, mrpt_xxx_EXPORTS


/*   The macros below for DLL import/export are required for Windows only.
    Mostly all the definitions in this file are copied or at least opengld
     on the file wx/dlimpexp.h, written by Vadim Zeitlin and published
	 under the wxWindows licence.
*/
#if defined(MRPT_OS_WINDOWS)
	#define MRPT_OPENGL_EXPORT __declspec(dllexport)
	#define MRPT_OPENGL_IMPORT __declspec(dllimport)
#else
#    define MRPT_OPENGL_EXPORT
#    define MRPT_OPENGL_IMPORT
#endif

/*  Macros that map to export declaration when building the DLL, to import
	declaration if using it or to nothing at all if we are not compiling as DLL */
#if defined(MRPT_BUILT_AS_DLL)
#	if defined(mrpt_opengl_EXPORTS)  /* Building the DLL */
#		define OPENGL_IMPEXP MRPT_OPENGL_EXPORT
#	else  /* Using the DLL */
#		define OPENGL_IMPEXP MRPT_OPENGL_IMPORT
#	endif
#else /* not making nor using DLL */
#    define OPENGL_IMPEXP
#endif

