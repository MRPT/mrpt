/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <vector>
#include <string>
#include <mrpt/base/link_pragmas.h>

namespace mrpt
{
	namespace system
	{
		/** Used in getCallStackBackTrace() */
		struct TCallStackEntry
		{
			inline TCallStackEntry() : address(NULL) {}
			/** Address of the calling point */
			void * address;
			/** Demangled symbol name */
			std::string symbolName;
			/** Original (before demangle) symbol name */
			std::string symbolNameOriginal;
		};

#ifdef _MSC_VER
		template class BASE_IMPEXP std::vector<TCallStackEntry>;
#endif

		/** See: getCallStackBackTrace() */
		struct BASE_IMPEXP TCallStackBackTrace
		{
			TCallStackBackTrace();
			std::vector<TCallStackEntry> backtrace_levels;
			
			/** Prints all backtrace entries, one per line in a human-readable format. */
			std::string asString() const;
		};

		/** Returns a list of strings representing the current call stack 
		  * backtrace. If possible, human-readable names are used for 
		  * functions.
		  */
		void BASE_IMPEXP getCallStackBackTrace(TCallStackBackTrace &out_bt);

	} // End of namespace
} // End of namespace
