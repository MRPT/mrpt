/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "core-precomp.h"  // Precompiled headers

#include <mrpt/core/format.h>
#include <cstdarg>

// A sprintf-like function for std::string
std::string mrpt::format(const char* fmt, ...)
{
	if (!fmt) return std::string();

	int result = -1, length = 2048;
	std::string buffer;
	while (result == -1)
	{
		buffer.resize(length);

		va_list args;  // This must be done WITHIN the loop
		va_start(args, fmt);
#if defined(_MSC_VER)
		result = ::vsnprintf_s(&buffer[0], length, _TRUNCATE, fmt, args);
#else
		result = ::vsnprintf(&buffer[0], length, fmt, args);
#endif
		va_end(args);

		// Truncated?
		if (result >= length) result = -1;
		length *= 2;

		// Ok?
		if (result >= 0)
		{
			buffer.resize(result);
		}
	}
	return buffer;
}
