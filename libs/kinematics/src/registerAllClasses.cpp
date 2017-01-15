/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "kinematics-precomp.h"  // Precompiled headers

#include <mrpt/kinematics.h>
#include <mrpt/utils/initializer.h>

using namespace mrpt::kinematics;
using namespace mrpt::utils;

MRPT_INITIALIZER(registerAllClasses_mrpt_kinematics)
{
#if !defined(DISABLE_MRPT_AUTO_CLASS_REGISTRATION)
	registerClass(CLASS_ID(CKinematicChain));

	// Vehicle vel cmds:
	registerClass(CLASS_ID(CVehicleVelCmd_DiffDriven));
	registerClass(CLASS_ID(CVehicleVelCmd_Holo));
#endif
}

