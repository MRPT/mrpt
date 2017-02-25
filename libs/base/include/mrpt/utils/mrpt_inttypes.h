/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef mrpt_inttypes_H
#define mrpt_inttypes_H

#include <mrpt/utils/mrpt_stdint.h>

#if HAVE_INTTYPES_H
#	ifndef __STDC_FORMAT_MACROS
#		define __STDC_FORMAT_MACROS
#	endif
#	include <inttypes.h>
#elif defined(_MSC_VER)
#	include	<mrpt/utils/msvc_inttypes.h>
#endif


#endif

