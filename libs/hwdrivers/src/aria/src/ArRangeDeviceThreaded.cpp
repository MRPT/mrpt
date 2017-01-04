/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "ArExport.h"
#include "ariaOSDef.h"
#include "ArRangeDeviceThreaded.h"

AREXPORT ArRangeDeviceThreaded::ArRangeDeviceThreaded(
	size_t currentBufferSize, size_t cumulativeBufferSize,
	const char *name, unsigned int maxRange) :
  ArRangeDevice(currentBufferSize, cumulativeBufferSize, name, maxRange),
  myRunThreadCB(this, &ArRangeDeviceThreaded::runThread),
  myTask(&myRunThreadCB)
{
  myTask.setThreadName(name);
}

AREXPORT ArRangeDeviceThreaded::~ArRangeDeviceThreaded()
{
}
