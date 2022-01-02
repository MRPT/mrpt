/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "core-precomp.h"  // Precompiled headers
//
#include <mrpt/core/winerror2str.h>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

std::string mrpt::winerror2str(const char* caller, const char* errorPrefix)
{
#ifdef _WIN32
	DWORD e = GetLastError();
	if (!e) return {};	// no error

	char s[2048];
	FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM, 0, e, 0, s, sizeof(s), nullptr);
	if (caller != nullptr)
	{
		std::string ret;
		ret = "[";
		ret += caller;
		ret += "]";
		if (errorPrefix) ret += errorPrefix;
		ret += s;
		return ret;
	}
	else
		return s;
#else
	return {};
#endif
}
