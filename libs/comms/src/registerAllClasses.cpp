/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "comms-precomp.h"
#include <mrpt/comms.h>

#include <mrpt/utils/initializer.h>

using namespace mrpt::comms;
using namespace mrpt::utils;

MRPT_INITIALIZER(registerAllClasses_mrpt_comms)
{
#if !defined(DISABLE_MRPT_AUTO_CLASS_REGISTRATION)
//	registerClass( CLASS_ID( XXXX ) );
#endif
}
