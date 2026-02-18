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
#include <mrpt/vision/registerAllClasses.h>
// deps:
#include <mrpt/obs/registerAllClasses.h>

MRPT_INITIALIZER(registerAllClasses_mrpt_vision)
{
  using namespace mrpt::vision;

#if !defined(DISABLE_MRPT_AUTO_CLASS_REGISTRATION)
  // No serializable classes to register in mrpt_vision 3.0
#endif
}

void mrpt::vision::registerAllClasses_mrpt_vision()
{
  ::registerAllClasses_mrpt_vision();
  // deps:
  mrpt::obs::registerAllClasses_mrpt_obs();
}
