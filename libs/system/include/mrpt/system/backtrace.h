/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/core/backtrace.h>

// Deprecated header, replace with <mrpt/core/backtrace.h>

namespace mrpt::system
{
using TCallStackBackTrace = mrpt::TCallStackBackTrace;

[[deprecated("Use mrpt::callStackBackTrace() instead")]]  //
inline void
	getCallStackBackTrace(TCallStackBackTrace& bt)
{
	mrpt::callStackBackTrace(bt);
}

[[deprecated("Use mrpt::callStackBackTrace() instead")]]  //
inline TCallStackBackTrace
	getCallStackBackTrace()
{
	return mrpt::callStackBackTrace();
}

};  // namespace mrpt::system
