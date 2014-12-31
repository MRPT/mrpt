/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once
#include <mrpt/obs/CObservationRGBD360.h>
MRPT_WARNING("*Deprecated header* Please replace with #include <mrpt/obs/CObservationRGBD360.h>. This backward compatible header will be removed in MRPT 2.0.0")
namespace mrpt { namespace slam {
	typedef mrpt::obs::CObservationRGBD360 CObservationRGBD360;    //!< Backward compatible typedef
	typedef mrpt::obs::CObservationRGBD360Ptr CObservationRGBD360Ptr; //!< Backward compatible typedef
} }
