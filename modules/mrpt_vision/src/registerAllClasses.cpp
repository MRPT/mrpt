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

#include "vision-precomp.h"  // Precompiled headers
//
#define MRPT_NO_WARN_BIG_HDR
#include <mrpt/core/initializer.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/vision.h>
#include <mrpt/vision/registerAllClasses.h>
// deps:
#include <mrpt/obs/registerAllClasses.h>

MRPT_INITIALIZER(registerAllClasses_mrpt_vision)
{
  using namespace mrpt::vision;
  using namespace mrpt::img;
  using namespace mrpt::obs;
  using namespace mrpt::maps;

#if !defined(DISABLE_MRPT_AUTO_CLASS_REGISTRATION)
  registerClass(CLASS_ID(CFeature));

  registerClass(CLASS_ID(CLandmark));
  registerClass(CLASS_ID(CLandmarksMap));

  registerClass(CLASS_ID(CObservationVisualLandmarks));
#endif
}

void mrpt::vision::registerAllClasses_mrpt_vision()
{
  ::registerAllClasses_mrpt_vision();
  // deps:
  mrpt::obs::registerAllClasses_mrpt_obs();
}
