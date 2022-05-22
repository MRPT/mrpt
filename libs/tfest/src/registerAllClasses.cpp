/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "tfest-precomp.h"
//
#include <mrpt/core/initializer.h>
#include <mrpt/tfest.h>
#include <mrpt/tfest/registerAllClasses.h>
// deps:
#include <mrpt/poses/registerAllClasses.h>

MRPT_INITIALIZER(registerAllClasses_mrpt_tfest)
{
	using namespace mrpt::tfest;

#if !defined(DISABLE_MRPT_AUTO_CLASS_REGISTRATION)
//	registerClass( CLASS_ID( XXXX ) );
#endif
}

void mrpt::tfest::registerAllClasses_mrpt_tfest()
{
	::registerAllClasses_mrpt_tfest();
	// deps:
	mrpt::poses::registerAllClasses_mrpt_poses();
}
