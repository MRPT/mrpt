/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "vision-precomp.h"   // Precompiled headers

#define MRPT_NO_WARN_BIG_HDR
#include <mrpt/vision.h>
#include <mrpt/utils/CSerializable.h>

#include <mrpt/utils/initializer.h>

using namespace mrpt::vision;
using namespace mrpt::utils;
using namespace mrpt::obs;
using namespace mrpt::maps;

MRPT_INITIALIZER(registerAllClasses_mrpt_vision)
{
#if !defined(DISABLE_MRPT_AUTO_CLASS_REGISTRATION)
	registerClass( CLASS_ID( CFeature ) );

	registerClass( CLASS_ID( CLandmark ) );
	registerClass( CLASS_ID( CLandmarksMap ) );

	registerClass( CLASS_ID( CObservationVisualLandmarks ) );
#endif
}

