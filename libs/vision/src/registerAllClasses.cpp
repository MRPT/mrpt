/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "vision-precomp.h"	 // Precompiled headers
//
#define MRPT_NO_WARN_BIG_HDR
#include <mrpt/core/initializer.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/vision.h>

using namespace mrpt::vision;
using namespace mrpt::img;
using namespace mrpt::obs;
using namespace mrpt::maps;

MRPT_INITIALIZER(registerAllClasses_mrpt_vision)
{
#if !defined(DISABLE_MRPT_AUTO_CLASS_REGISTRATION)
	registerClass(CLASS_ID(CFeature));

	registerClass(CLASS_ID(CLandmark));
	registerClass(CLASS_ID(CLandmarksMap));

	registerClass(CLASS_ID(CObservationVisualLandmarks));
#endif
}
