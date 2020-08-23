/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "core-precomp.h"  // Precompiled headers
//
#include <mrpt/core/exceptions.h>

namespace mrpt::internal
{
std::string exception_line_msg(
	const std::string_view& msg, const char* filename, unsigned int line,
	const char* function_name)
{
	std::string s;
	s += filename;
	s += ":";
	s += std::to_string(line);
	s += ": [";
	s += function_name;
	s += "] ";
	s += msg;
	s += "\n";
	return s;
}

static size_t findClosingBracket(
	const char chClosing, const char chOpening, const std::string& str)
{
	const size_t N = str.size();
	int nestedLevel = 0;
	for (size_t p = 0; p < N; p++)
	{
		if (str[p] == chClosing)
		{
			if (nestedLevel == 1) return p;
			nestedLevel--;
		}
		else if (str[p] == chOpening)
			nestedLevel++;
	}
	return std::string::npos;
}

/** Recursive implementation for mrpt::exception_to_str() */
void impl_excep_to_str(const std::exception& e, std::string& ret, int lvl = 0)
{
	using namespace std::string_literals;
	std::string err{e.what()};
	if (!err.empty() && *err.rbegin() != '\n') err += "\n"s;
	ret = "["s + std::to_string(lvl) + "] "s + err + ret;
	try
	{
		std::rethrow_if_nested(e);
		// We traversed the entire call stack,
		// show just the original error message: "file:line: [func] MSG"
		if (const auto idx = findClosingBracket(']', '[', err);
			idx != std::string::npos)
			err = "Exception message:"s + err.substr(idx + 1);
		ret = err + std::string("==== MRPT exception backtrace ====\n") + ret;
	}
	catch (const std::exception& er)
	{
		// It's nested: recursive call
		impl_excep_to_str(er, ret, lvl + 1);
	}
	catch (...)
	{
	}
}
}  // namespace mrpt::internal

std::string mrpt::exception_to_str(const std::exception& e)
{
	std::string descr;
	mrpt::internal::impl_excep_to_str(e, descr);
	return descr;
}
