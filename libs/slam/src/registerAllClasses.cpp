/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "slam-precomp.h"   // Precompiled headers

#define MRPT_NO_WARN_BIG_HDR
#include <mrpt/slam.h>

#include <mrpt/utils/initializer.h>

using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::maps;
using namespace mrpt::opengl;

MRPT_INITIALIZER(registerAllClasses_mrpt_core)
{
#if !defined(DISABLE_MRPT_AUTO_CLASS_REGISTRATION)
//   Hack to enable compatibility with an older name of this class:
	registerClass( CLASS_ID( CMultiMetricMap ) );
	registerClassCustomName( "CHybridMetricMap", CLASS_ID( CMultiMetricMap ) );

	registerClass( CLASS_ID( CIncrementalMapPartitioner ) );
	registerClass( CLASS_ID( CMultiMetricMapPDF ) );
#endif
}

