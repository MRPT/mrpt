/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef mrpt_stdint_H
#define mrpt_stdint_H

#include <mrpt/config.h>

// Define macros in platform dependant stdint.h header:
#ifndef __STDC_FORMAT_MACROS
#	define __STDC_FORMAT_MACROS
#endif
#ifndef __STDC_CONSTANT_MACROS
#	define __STDC_CONSTANT_MACROS
#endif
#ifndef __STDC_LIMIT_MACROS
#	define __STDC_LIMIT_MACROS
#endif

// Standard elemental types:
#if HAVE_STDINT_H
#	include <stdint.h>
#else
#	include "pstdint.h"  // The "portable stdint header file"
#endif



#endif

