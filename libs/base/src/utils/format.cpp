/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers


#include <mrpt/utils/utils_defs.h>
#include <mrpt/system/os.h>

#include <cstdarg>

using namespace mrpt;
using namespace mrpt::system;
using namespace std;


// A sprintf-like function for std::string 
string mrpt::format(const char *fmt, ...)
{
	if (!fmt) return string("");

	int   result = -1, length = 1024;
	vector<char> buffer;
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
	string s(&buffer[0]);
	return s;
}

