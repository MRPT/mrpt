/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include "apps-precomp.h"  // Precompiled headers
//
#include <mrpt/apps/registerAllClasses.h>
#include <mrpt/core/initializer.h>
// Deps:
#include <mrpt/gui/registerAllClasses.h>
#include <mrpt/hwdrivers/registerAllClasses.h>
#include <mrpt/slam/registerAllClasses.h>
#include <mrpt/topography/registerAllClasses.h>

MRPT_INITIALIZER(registerAllClasses_mrpt_apps)
{
#if !defined(DISABLE_MRPT_AUTO_CLASS_REGISTRATION)
  // registerClass(CLASS_ID(XXX));
#endif
}

void mrpt::apps::registerAllClasses_mrpt_apps()
{
  ::registerAllClasses_mrpt_apps();
  mrpt::gui::registerAllClasses_mrpt_gui();
  mrpt::hwdrivers::registerAllClasses_mrpt_hwdrivers();
  mrpt::slam::registerAllClasses_mrpt_slam();
  mrpt::topography::registerAllClasses_mrpt_topography();
}
