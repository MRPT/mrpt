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
#include <mrpt/system/memory.h>

#include <cstdarg>

using namespace mrpt;
using namespace mrpt::system;
using namespace std;


// A sprintf-like function for std::string 
string mrpt::format(const char *fmt, ...)
{
	if (!fmt) return string("");

	int   result = -1, length = 2048;
	char *buffer=nullptr;
	while (result == -1)
	{
		if (buffer!=nullptr) {
			mrpt_alloca_free(buffer);
		}
		buffer= static_cast<char*>(mrpt_alloca(length+10));
		if (buffer==nullptr) THROW_EXCEPTION("mrpt::format(): Out of memory?");

		va_list args;  // This must be done WITHIN the loop
		va_start(args,fmt);
		result = os::vsnprintf(buffer, length, fmt, args);
		va_end(args);

		// Truncated?
		if (result>=length) result=-1;
		length*=2;
	}

#if defined(HAVE_ALLOCA)
	// We are safe to directly return without an explicit alloca_free():
	return string(buffer);
#else
	string s(buffer);
	mrpt_alloca_free(buffer);
	return s;
#endif
}

