/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "core-precomp.h"  // Precompiled headers
//
#include <mrpt/core/demangle.h>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
//
#include <DbgHelp.h>
#else
#include <cxxabi.h>	 // __cxa_demangle()
#include <dlfcn.h>	// dladdr()
#include <execinfo.h>

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#endif

std::string mrpt::demangle(const std::string& symbolName)
{
	if (symbolName.empty()) return {};

#if defined(_WIN32)
	char undecorated_name[1024];
	if (!UnDecorateSymbolName(
			symbolName.c_str(), undecorated_name, sizeof(undecorated_name),
			UNDNAME_COMPLETE))
	{ return symbolName; }
	else
	{
		return std::string(undecorated_name);
	}
#else
	char* demangled = nullptr;
	int status = -1;
	demangled =
		abi::__cxa_demangle(symbolName.c_str(), nullptr, nullptr, &status);
	std::string ret = status == 0 ? std::string(demangled) : symbolName;
	free(demangled);
	return ret;
#endif
}
