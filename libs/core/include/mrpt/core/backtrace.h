/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <string>
#include <vector>

namespace mrpt
{
/** \addtogroup mrpt_core_grp
 * @{ */

/** Used in callStackBackTrace() */
struct TCallStackEntry
{
	TCallStackEntry() = default;

	/** Address of the calling point */
	void* address = nullptr;

	/** Demangled symbol name (empty if not available) */
	std::string symbolName;
	/** Original (before demangle) symbol name  (empty if not available) */
	std::string symbolNameOriginal;

	std::string sourceFileName;
	int sourceFileNumber = 0;
};

#ifdef _MSC_VER
template class std::vector<TCallStackEntry>;
#endif

/** See: callStackBackTrace() */
struct TCallStackBackTrace
{
	TCallStackBackTrace();
	std::vector<TCallStackEntry> backtrace_levels;

	/** Prints all backtrace entries, one per line in a human-readable format.
	 */
	std::string asString() const noexcept;
};

/** Returns a list of strings representing the current call stack
 * backtrace. If possible, human-readable names are used for
 * functions.
 *
 * \note (Moved from mrpt-system to mrpt-core in MRPT 2.1.5)
 */
void callStackBackTrace(TCallStackBackTrace& out_bt) noexcept;

inline TCallStackBackTrace callStackBackTrace() noexcept
{
	TCallStackBackTrace bt;
	callStackBackTrace(bt);
	return bt;
}

/** @} */

}  // namespace mrpt
