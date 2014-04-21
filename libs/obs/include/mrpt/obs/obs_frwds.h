/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

// Forward declarations for the library "mrpt-obs"

namespace mrpt
{
	namespace slam
	{
		class CObservation; struct CObservationPtr;
		class CSensoryFrame; struct CSensoryFramePtr;
		class CMetricMap; struct CMetricMapPtr;
		class CPointsMap;
		class CSimplePointsMap;
		class CSimpleMap;

	}

	namespace poses
	{
		class CPose3DPDF; struct CPose3DPDFPtr;
	}
}
