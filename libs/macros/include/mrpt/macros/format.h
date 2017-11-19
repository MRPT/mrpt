/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/macros/common.h>

/** This is the global namespace for all Mobile Robot Programming Toolkit (MRPT)
 * libraries. */
namespace mrpt
{
/** A std::string version of C sprintf.
  *  You can call this to obtain a std::string using printf-like syntax.
  */
std::string format(const char* fmt, ...) MRPT_printf_format_check(1, 2);
}