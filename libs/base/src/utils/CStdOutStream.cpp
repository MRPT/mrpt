/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers 


#include <mrpt/utils/CStdOutStream.h>
using namespace mrpt::utils;

#include <iostream>


size_t CStdOutStream::Write(const void *Buffer,size_t Count)
{
	// Assume we'll always receive valid NULL-terminated strings:
	std::cout << static_cast<const char*>(Buffer);
	return Count;
}
