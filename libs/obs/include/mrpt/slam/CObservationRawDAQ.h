/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once
#include <mrpt/obs/CObservationRawDAQ.h>
MRPT_WARNING("*Deprecated header* Please replace with #include <mrpt/obs/CObservationRawDAQ.h>. This backward compatible header will be removed in MRPT 2.0.0")
namespace mrpt { namespace slam {
	using mrpt::obs::CObservationRawDAQ;    //!< Backward compatibility
	using mrpt::obs::CObservationRawDAQPtr; //!< Backward compatibility
} }
