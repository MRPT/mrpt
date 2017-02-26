/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef AREXPORT_H
#define AREXPORT_H

#if defined(_WIN32) || defined(WIN32)

#ifndef SWIG
#ifndef ARIA_STATIC
	#undef AREXPORT
	#ifdef ARIADLL_EXPORTS
		#define AREXPORT _declspec(dllexport)
	#else
		#define AREXPORT _declspec(dllimport)
	#endif
#else // ARIA_STATIC
	#define AREXPORT
#endif // ARIA_STATIC
#endif // SWIG

#else // WIN32

#define AREXPORT

#endif // WIN32

#endif // AREXPORT_H


