/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <vector>
#include <string>

namespace mrpt::system
{
/** Used in getCallStackBackTrace() */
struct TCallStackEntry
{
	inline TCallStackEntry() : address(nullptr) {}
	/** Address of the calling point */
	void* address;
	/** Demangled symbol name */
	std::string symbolName;
	/** Original (before demangle) symbol name */
	std::string symbolNameOriginal;
};

#ifdef _MSC_VER
template class std::vector<TCallStackEntry>;
#endif

/** See: getCallStackBackTrace() */
struct TCallStackBackTrace
{
	TCallStackBackTrace();
	std::vector<TCallStackEntry> backtrace_levels;

	/** Prints all backtrace entries, one per line in a human-readable format.
	 */
	std::string asString() const;
};

/** Returns a list of strings representing the current call stack
 * backtrace. If possible, human-readable names are used for
 * functions.
 */
void getCallStackBackTrace(TCallStackBackTrace& out_bt);

}  // namespace mrpt::system
