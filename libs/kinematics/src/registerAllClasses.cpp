/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "kinematics-precomp.h"	 // Precompiled headers
//
#include <mrpt/core/initializer.h>
#include <mrpt/kinematics.h>
#include <mrpt/kinematics/registerAllClasses.h>
// deps:
#include <mrpt/opengl/registerAllClasses.h>

MRPT_INITIALIZER(registerAllClasses_mrpt_kinematics)
{
	using namespace mrpt::kinematics;

#if !defined(DISABLE_MRPT_AUTO_CLASS_REGISTRATION)
	registerClass(CLASS_ID(CKinematicChain));

	// Vehicle vel cmds:
	registerClass(CLASS_ID(CVehicleVelCmd_DiffDriven));
	registerClass(CLASS_ID(CVehicleVelCmd_Holo));
#endif
}

void mrpt::kinematics::registerAllClasses_mrpt_kinematics()
{
	::registerAllClasses_mrpt_kinematics();
	// deps:
	mrpt::opengl::registerAllClasses_mrpt_opengl();
}
