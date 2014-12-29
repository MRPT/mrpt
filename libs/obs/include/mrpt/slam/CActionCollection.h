/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once
#include <mrpt/obs/CActionCollection.h>
MRPT_WARNING("*Deprecated header* Please replace with #include <mrpt/obs/CActionCollection.h>. This backward compatible header will be removed in MRPT 2.0.0")
namespace mrpt { namespace slam {
	typedef mrpt::obs::CActionCollection CActionCollection;    //!< Backward compatible typedef
	typedef mrpt::obs::CActionCollectionPtr CActionCollectionPtr; //!< Backward compatible typedef
} }
