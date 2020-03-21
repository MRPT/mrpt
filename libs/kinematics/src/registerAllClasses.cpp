/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "kinematics-precomp.h"  // Precompiled headers

#include <mrpt/core/initializer.h>
#include <mrpt/kinematics.h>

using namespace mrpt::kinematics;

MRPT_INITIALIZER(registerAllClasses_mrpt_kinematics)
{
#if !defined(DISABLE_MRPT_AUTO_CLASS_REGISTRATION)
	registerClass(CLASS_ID(CKinematicChain));

	// Vehicle vel cmds:
	registerClass(CLASS_ID(CVehicleVelCmd_DiffDriven));
	registerClass(CLASS_ID(CVehicleVelCmd_Holo));
#endif
}
