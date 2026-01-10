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
