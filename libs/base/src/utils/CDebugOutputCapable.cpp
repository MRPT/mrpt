/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#define MRPT_DISABLE_WARN_DEPRECATED_DEBUG_OUTPUT_CAPABLE
#include <mrpt/utils/CDebugOutputCapable.h>
#include <mrpt/system/memory.h>
#include <mrpt/system/os.h>

#include <vector>
#include <cstdarg>

using namespace mrpt::utils;
using namespace mrpt::system;

/*---------------------------------------------------------------
					printf_debug
 ---------------------------------------------------------------*/
void CDebugOutputCapable::printf_debug( const char *fmt, ... ) const
{
	if (!fmt) return;

	int   result = -1, length = 1024;
	std::vector<char> buffer;
	while (result == -1)
	{
		buffer.resize(length + 10);

		va_list args;  // This must be done WITHIN the loop
		va_start(args,fmt);
		result = os::vsnprintf(&buffer[0], length, fmt, args);
		va_end(args);

		// Truncated?
		if (result>=length) result=-1;
		length*=2;
	}

	// Output:
	std::string s(&buffer[0]);
	COutputLogger::logStr(LVL_INFO, s);
}
