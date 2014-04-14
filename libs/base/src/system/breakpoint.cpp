/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/utils/utils_defs.h>
#include <mrpt/system/os.h>

/** Only when built in debug (with _DEBUG), this function will be called just before raising any MRPT exception,
  *  so the user can conveniently put a breakpoint here to explore the call stack, etc.
  */
void mrpt::system::breakpoint(const std::string &exception_msg)
{
	// Does nothing, but provides a place where to put a breakpoint:
	exception_msg.size();
}
