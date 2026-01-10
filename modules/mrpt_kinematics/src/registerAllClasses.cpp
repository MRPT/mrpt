/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <mrpt/core/initializer.h>
#include <mrpt/kinematics.h>
#include <mrpt/kinematics/registerAllClasses.h>
// deps:
#include <mrpt/viz/registerAllClasses.h>

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
  mrpt::viz::registerAllClasses_mrpt_viz();
}
