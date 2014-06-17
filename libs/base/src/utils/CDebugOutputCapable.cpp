/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/utils/CDebugOutputCapable.h>
#include <mrpt/system/memory.h>
#include <mrpt/system/os.h>

#ifdef MRPT_OS_WINDOWS
	#define WIN32_LEAN_AND_MEAN
	#include <windows.h>
#endif

#include <vector>
#include <cstdarg>
#include <iostream>

using namespace mrpt::utils;
using namespace mrpt::system;


/*---------------------------------------------------------------
					printf_debug
 ---------------------------------------------------------------*/
void CDebugOutputCapable::printf_debug( const char *fmt, ... )
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
	std::cout << &buffer[0];

#ifdef _MSC_VER
	OutputDebugStringA(&buffer[0]);
#endif
}
